#!/usr/bin/env python3
# dlna_live.py — C3X DLNA/UPnP 行车画面直播源 (供 MMI 3G+ WiFi 播放器)
# 由 manager 按 DlnaLiveEnabled 参数门控启动 (开发者选项里有开关)。
#
# 功能:
#   1. 打开 IsLiveStreaming → 订阅 livestreamWideRoadEncodeData (广角前路 H264 1152x720)
#   2. PyAV 转码 → MPEG-2 PS 720x480@20 (MMI parser_mpgvfile + IS_MPEG2_Dec_C64x 硬解)
#   3. HTTP :8200 伺服 /live.mpg (无限流) + rootDesc/SCPD/Browse
#   4. SSDP :1900 应答 M-SEARCH + 周期 NOTIFY alive
#   5. offroad 看门狗: 熄火宽限 60s 自动退出, 复位 DlnaLiveEnabled 总开关
#
# 依赖: pip install --target=/data/pylibs av (一次性, /data 持久)

import atexit
import json
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, "/data/pylibs")
sys.path.insert(0, "/data/openpilot")

HTTP_PORT = 8200
DEVICE_UUID = "3fa0c3c3-0000-4000-8000-c33c33c33001"  # 固定 UUID
FRIENDLY_NAME = "C3X-LiveCam"
V4L2_KEYFRAME = 0x0008

# ---------- 工具 ----------

def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.100", 80))  # 不真发包, 只为选出接口(车载热点网关/MMI)
        return s.getsockname()[0]
    except OSError:
        try:
            s.connect(("8.8.8.8", 53))
            return s.getsockname()[0]
        except OSError:
            return "127.0.0.1"
    finally:
        s.close()


# ---------- 转码管道 (单客户端, 连接时重启以保证 PAT/PMT 在流头) ----------

class LivePipeline:
    def __init__(self):
        self.sink_queues = []
        self.thread = None
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self._pts = 0

    def start(self, q):
        self.stop()  # 重启以保证新客户端从 PAT/PMT+关键帧开始
        self.stop_flag.clear()
        self.sink_queues = [q]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_flag.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5)
        self.thread = None
        self.sink_queues = []

    def _broadcast(self, data: bytes):
        for q in list(self.sink_queues):
            try:
                if q.full():
                    q.get_nowait()  # 丢旧保新, 直播语义
                q.put_nowait(data)
            except queue.Empty:
                pass

    def _run(self):
        import av
        from openpilot.cereal import messaging

        class Sink:
            def write(s, b):
                self._broadcast(bytes(b))
                return len(b)
            def writable(s): return True
            def seekable(s): return False
            def readable(s): return False

        try:
            sock = messaging.sub_sock("livestreamWideRoadEncodeData", conflate=True, timeout=2000)
        except Exception as e:
            print(f"[pipe] 订阅失败: {e}", flush=True)
            return

        try:
            dec = av.CodecContext.create("h264", "r")
            dec.thread_type = "SLICE"  # 帧级多线程会按线程数攒帧, 改 slice 级降解码延迟
            out = av.open(Sink(), mode="w", format="mpeg", options={"flush_packets": "1"})
            vstream = out.add_stream("mpeg2video", rate=20, options={"bf": "0", "trellis": "0"})
            vstream.width = 720
            vstream.height = 480
            vstream.pix_fmt = "yuv420p"
            vstream.bit_rate = 2_500_000
            out.start_encoding()

            sent_header = False
            self._pts = 0
            print("[pipe] 转码开始", flush=True)
            while not self.stop_flag.is_set():
                msg = messaging.recv_one(sock)
                if msg is None:
                    continue
                ev = getattr(msg, "livestreamWideRoadEncodeData", None)
                if ev is None:
                    continue
                if not sent_header:
                    if not (ev.idx.flags & V4L2_KEYFRAME):
                        continue
                    # SPS/PPS 单独成包 h264 解码器拒收, 必须和首个 IDR 拼成一包
                    payload = bytes(ev.header) + bytes(ev.data)
                    sent_header = True
                else:
                    payload = bytes(ev.data)
                for frame in dec.decode(av.packet.Packet(payload)):
                    f2 = frame.reformat(width=720, height=480, format="yuv420p")
                    f2.pts = self._pts
                    self._pts += 1
                    for pkt in vstream.encode(f2):
                        out.mux(pkt)
        except Exception as e:
            print(f"[pipe] 异常退出: {type(e).__name__}: {e}", flush=True)


pipeline = LivePipeline()


# ---------- HTTP 服务 ----------

ROOTDESC = """<?xml version="1.0"?>
<root xmlns="urn:schemas-upnp-org:device-1-0"><specVersion><major>1</major><minor>0</minor></specVersion>
<device><deviceType>urn:schemas-upnp-org:device:MediaServer:1</deviceType>
<friendlyName>%s</friendlyName><manufacturer>comma bridge</manufacturer>
<modelName>C3X Live Camera</modelName><modelNumber>1</modelNumber>
<UDN>uuid:%s</UDN>
<serviceList>
<service><serviceType>urn:schemas-upnp-org:service:ContentDirectory:1</serviceType>
<serviceId>urn:upnp-org:serviceId:ContentDirectory</serviceId>
<controlURL>/ctl/ContentDir</controlURL><eventSubURL>/evt/ContentDir</eventSubURL>
<SCPDURL>/ContentDir.xml</SCPDURL></service>
<service><serviceType>urn:schemas-upnp-org:service:ConnectionManager:1</serviceType>
<serviceId>urn:upnp-org:serviceId:ConnectionManager</serviceId>
<controlURL>/ctl/ConnectionMgr</controlURL><eventSubURL>/evt/ConnectionMgr</eventSubURL>
<SCPDURL>/ConnectionMgr.xml</SCPDURL></service>
</serviceList></device></root>"""

CDS_SCPD = """<?xml version="1.0"?>
<scpd xmlns="urn:schemas-upnp-org:service-1-0"><specVersion><major>1</major><minor>0</minor></specVersion>
<actionList>
<action><name>Browse</name><argumentList>
<argument><name>ObjectID</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_ObjectID</relatedStateVariable></argument>
<argument><name>BrowseFlag</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_BrowseFlag</relatedStateVariable></argument>
<argument><name>Filter</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_Filter</relatedStateVariable></argument>
<argument><name>StartingIndex</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_Index</relatedStateVariable></argument>
<argument><name>RequestedCount</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_Count</relatedStateVariable></argument>
<argument><name>SortCriteria</name><direction>in</direction><relatedStateVariable>A_ARG_TYPE_SortCriteria</relatedStateVariable></argument>
<argument><name>Result</name><direction>out</direction><relatedStateVariable>A_ARG_TYPE_Result</relatedStateVariable></argument>
<argument><name>NumberReturned</name><direction>out</direction><relatedStateVariable>A_ARG_TYPE_Count</relatedStateVariable></argument>
<argument><name>TotalMatches</name><direction>out</direction><relatedStateVariable>A_ARG_TYPE_Count</relatedStateVariable></argument>
<argument><name>UpdateID</name><direction>out</direction><relatedStateVariable>A_ARG_TYPE_UpdateID</relatedStateVariable></argument>
</argumentList></action>
<action><name>GetSystemUpdateID</name><argumentList>
<argument><name>Id</name><direction>out</direction><relatedStateVariable>SystemUpdateID</relatedStateVariable></argument>
</argumentList></action>
<action><name>GetSearchCapabilities</name><argumentList>
<argument><name>SearchCaps</name><direction>out</direction><relatedStateVariable>SearchCapabilities</relatedStateVariable></argument>
</argumentList></action>
<action><name>GetSortCapabilities</name><argumentList>
<argument><name>SortCaps</name><direction>out</direction><relatedStateVariable>SortCapabilities</relatedStateVariable></argument>
</argumentList></action>
</actionList>
<serviceStateTable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_ObjectID</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_BrowseFlag</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_Filter</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_Index</name><dataType>ui4</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_Count</name><dataType>ui4</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_SortCriteria</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_Result</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>A_ARG_TYPE_UpdateID</name><dataType>ui4</dataType></stateVariable>
<stateVariable sendEvents="yes"><name>SystemUpdateID</name><dataType>ui4</dataType></stateVariable>
<stateVariable sendEvents="no"><name>SearchCapabilities</name><dataType>string</dataType></stateVariable>
<stateVariable sendEvents="no"><name>SortCapabilities</name><dataType>string</dataType></stateVariable>
</serviceStateTable></scpd>"""

CM_SCPD = """<?xml version="1.0"?>
<scpd xmlns="urn:schemas-upnp-org:service-1-0"><specVersion><major>1</major><minor>0</minor></specVersion>
<actionList>
<action><name>GetCurrentConnectionIDs</name><argumentList>
<argument><name>ConnectionIDs</name><direction>out</direction><relatedStateVariable>CurrentConnectionIDs</relatedStateVariable></argument>
</argumentList></action>
</actionList>
<serviceStateTable>
<stateVariable sendEvents="yes"><name>CurrentConnectionIDs</name><dataType>string</dataType></stateVariable>
</serviceStateTable></scpd>"""


def soap_wrap(action, service, inner):
    return ('<?xml version="1.0" encoding="utf-8"?>'
            '<s:Envelope xmlns:s="http://schemas.xmlsoap.org/soap/envelope/" '
            's:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">'
            f'<s:Body><u:{action}Response xmlns:u="urn:schemas-upnp-org:service:{service}:1">'
            f'{inner}</u:{action}Response></s:Body></s:Envelope>')


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "C3XLive/1.0"
    adv_ip = "127.0.0.1"

    def log_message(self, *a):
        print(f"[http] {self.client_address[0]} {self.path}", flush=True)

    def _send(self, code, body: bytes, ctype="text/xml; charset=utf-8", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("EXT", "")
        if extra:
            for k, v in extra.items():
                self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def do_GET(self):
        if self.path == "/live.mpg":
            if self.command == "HEAD":
                self.send_response(200)
                self.send_header("Content-Type", "video/mpeg")
                self.send_header("Connection", "close")
                self.end_headers()
            else:
                self._stream_live()
        elif self.path == "/rootDesc.xml":
            self._send(200, (ROOTDESC % (FRIENDLY_NAME, DEVICE_UUID)).encode())
        elif self.path == "/ContentDir.xml":
            self._send(200, CDS_SCPD.encode())
        elif self.path == "/ConnectionMgr.xml":
            self._send(200, CM_SCPD.encode())
        else:
            self.send_error(404)

    def do_HEAD(self):
        self.do_GET()

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n).decode(errors="replace") if n else ""
        action = self.headers.get("SOAPACTION", "")
        if self.path == "/ctl/ContentDir":
            if "Browse" in action:
                self._browse(body)
            elif "GetSystemUpdateID" in action:
                self._send(200, soap_wrap("GetSystemUpdateID", "ContentDirectory", "<Id>1</Id>").encode())
            elif "GetSearchCapabilities" in action:
                self._send(200, soap_wrap("GetSearchCapabilities", "ContentDirectory", "<SearchCaps></SearchCaps>").encode())
            elif "GetSortCapabilities" in action:
                self._send(200, soap_wrap("GetSortCapabilities", "ContentDirectory", "<SortCaps></SortCaps>").encode())
            else:
                self.send_error(400)
        elif self.path == "/ctl/ConnectionMgr":
            self._send(200, soap_wrap("GetCurrentConnectionIDs", "ConnectionManager", "<ConnectionIDs>0</ConnectionIDs>").encode())
        else:
            self.send_error(404)

    def _browse(self, body):
        import re
        oid = (re.search(r"<ObjectID>(.*?)</ObjectID>", body) or [None, "0"])[1]
        flag = (re.search(r"<BrowseFlag>(.*?)</BrowseFlag>", body) or [None, "BrowseDirectChildren"])[1]
        base = f"http://{self.adv_ip}:{HTTP_PORT}"

        item_didl = (
            f'<item id="i1" parentID="c1" restricted="1">'
            f'<dc:title xmlns:dc="http://purl.org/dc/elements/1.1/">C3X 行车直播</dc:title>'
            f'<upnp:class xmlns:upnp="urn:schemas-upnp-org:metadata-1-0/upnp/">object.item.videoItem</upnp:class>'
            f'<res protocolInfo="http-get:*:video/mpeg:*" duration="0:00:00">{base}/live.mpg</res>'
            f'</item>')
        container_didl = (
            f'<container id="c1" parentID="0" restricted="1" childCount="1">'
            f'<dc:title xmlns:dc="http://purl.org/dc/elements/1.1/">Live</dc:title>'
            f'<upnp:class xmlns:upnp="urn:schemas-upnp-org:metadata-1-0/upnp/">object.container.storageFolder</upnp:class>'
            f'</container>')
        root_meta = (
            f'<container id="0" parentID="-1" restricted="1" childCount="1">'
            f'<dc:title xmlns:dc="http://purl.org/dc/elements/1.1/">root</dc:title>'
            f'<upnp:class xmlns:upnp="urn:schemas-upnp-org:metadata-1-0/upnp/">object.container</upnp:class>'
            f'</container>')

        if flag == "BrowseMetadata":
            inner_xml = root_meta if oid == "0" else (container_didl if oid == "c1" else item_didl)
            total = 1
        else:  # BrowseDirectChildren
            if oid == "0":
                inner_xml, total = container_didl, 1
            elif oid == "c1":
                inner_xml, total = item_didl, 1
            else:
                inner_xml, total = "", 0

        didl = ('<DIDL-Lite xmlns="urn:schemas-upnp-org:metadata-1-0/DIDL-Lite/" '
                'xmlns:dc="http://purl.org/dc/elements/1.1/" '
                'xmlns:upnp="urn:schemas-upnp-org:metadata-1-0/upnp/">'
                f'{inner_xml}</DIDL-Lite>')
        inner = f"<Result>{esc(didl)}</Result><NumberReturned>{total}</NumberReturned><TotalMatches>{total}</TotalMatches><UpdateID>1</UpdateID>"
        self._send(200, soap_wrap("Browse", "ContentDirectory", inner).encode())

    def _stream_live(self):
        q = queue.Queue(maxsize=30)  # 积压上限 30 块, 多了直接丢旧保新
        print(f"[live] MMI/客户端接入 {self.client_address[0]}, 启动转码", flush=True)
        pipeline.start(q)
        try:
            self.send_response(200)
            self.send_header("Content-Type", "video/mpeg")
            self.send_header("Connection", "close")
            self.end_headers()
            while True:
                try:
                    chunk = q.get(timeout=10)
                except queue.Empty:
                    chunk = b""
                if chunk:
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            pipeline.stop()
            print(f"[live] 客户端断开, 转码停止", flush=True)


# ---------- SSDP ----------

def ssdp_responder(adv_ip_holder):
    MCAST = ("239.255.255.250", 1900)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 1900))
    mreq = struct.pack("4s4s", socket.inet_aton(MCAST[0]), socket.inet_aton("0.0.0.0"))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    s.settimeout(2)
    print("[ssdp] 监听 M-SEARCH", flush=True)
    while True:
        try:
            data, addr = s.recvfrom(4096)
        except socket.timeout:
            adv_ip_holder[0] = local_ip()
            continue
        if b"M-SEARCH" not in data:
            continue
        print(f"[ssdp] 收到 M-SEARCH ← {addr[0]}", flush=True)
        ip = adv_ip_holder[0]
        for st in (b"ssdp:all", b"upnp:rootdevice", b"urn:schemas-upnp-org:device:MediaServer:1"):
            if st in data:
                usn = f"uuid:{DEVICE_UUID}"
                nts = "upnp:rootdevice" if st == b"upnp:rootdevice" else (
                      "urn:schemas-upnp-org:device:MediaServer:1" if b"MediaServer" in st else usn)
                resp = (f"HTTP/1.1 200 OK\r\nCACHE-CONTROL: max-age=1800\r\nEXT:\r\n"
                        f"LOCATION: http://{ip}:{HTTP_PORT}/rootDesc.xml\r\n"
                        f"SERVER: QNX/6.3 UPnP/1.0 C3XLive/1.0\r\n"
                        f"ST: {nts}\r\nUSN: {usn}::{nts if nts != usn else 'upnp:rootdevice'}\r\n\r\n")
                try:
                    s.sendto(resp.encode(), addr)
                    print(f"[ssdp] 应答 M-SEARCH ← {addr[0]}", flush=True)
                except OSError:
                    pass
                break


def ssdp_alive(adv_ip_holder):
    time.sleep(5)
    while True:
        ip = adv_ip_holder[0]
        for nt in ("upnp:rootdevice", f"uuid:{DEVICE_UUID}", "urn:schemas-upnp-org:device:MediaServer:1"):
            msg = (f"NOTIFY * HTTP/1.1\r\nHOST: 239.255.255.250:1900\r\n"
                   f"CACHE-CONTROL: max-age=1800\r\n"
                   f"LOCATION: http://{ip}:{HTTP_PORT}/rootDesc.xml\r\n"
                   f"NT: {nt}\r\nNTS: ssdp:alive\r\n"
                   f"SERVER: QNX/6.3 UPnP/1.0 C3XLive/1.0\r\n"
                   f"USN: uuid:{DEVICE_UUID}::{nt}\r\n\r\n")
            try:
                t = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                t.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 4)
                t.sendto(msg.encode(), ("239.255.255.250", 1900))
                t.close()
            except OSError:
                pass
        time.sleep(30)


# ---------- 主流程 ----------

# 全局退出标志: 任何退出路径置位, 阻止 ip_refresher 等后台线程再写 IsLiveStreaming
_EXITING = threading.Event()

_PARAM_DIR = "/data/params/d"


def _write_param(name: str, value: str) -> None:
  with open(f"{_PARAM_DIR}/{name}", "w") as f:
    f.write(value)


def cleanup_live_stream() -> None:
  """复位 IsLiveStreaming=0, 让 manager teardown camerad/encoderd, 防 ISP 持续满速。
  SIGTERM/SIGINT handler、atexit、serve_forever finally 四路退出都会走到这里。"""
  _EXITING.set()
  try:
    _write_param("IsLiveStreaming", "0")
    print("[main] cleanup: IsLiveStreaming=0", flush=True)
  except OSError as e:
    print(f"[main] cleanup 写 IsLiveStreaming 失败: {e}", flush=True)


# ---------- offroad 看门狗 ----------
# 语义: offroad 连续累计 OFFROAD_GRACE_SEC 秒 -> 自动退出 + 复位 DlnaLiveEnabled 总开关为 0
#   - 用户要求: "offroad 状态下 DLNA 播放 1 分钟就自动退出, 开关也置为关; 开关默认关, 必须手动打开"
#   - 开关复位后 UI 状态与实际一致, manager 下一轮 ensure_running 按开关状态正常回收进程
#   - 进程启动时就处于 offroad (停车开开关/停车重启) 同样从启动起计时, 即停车态最多播 60s
#   - 无"全局硬超时": 行车中直播不应被无条件掐断 (2026-09-05 审查 minimax 初版时移除)
OFFROAD_PARAMS_POLL_SEC = 2
OFFROAD_GRACE_SEC = 60


def _self_terminate() -> None:
  # 先复位总开关再自杀; 之后 os.kill 走 SIGTERM handler -> cleanup_live_stream -> IsLiveStreaming=0
  try:
    _write_param("DlnaLiveEnabled", "0")
    print("[offroad_wd] DlnaLiveEnabled 已复位为 0", flush=True)
  except OSError as e:
    print(f"[offroad_wd] 复位 DlnaLiveEnabled 失败: {e}", flush=True)
  os.kill(os.getpid(), signal.SIGTERM)


def _read_is_offroad() -> bool:
  try:
    with open(f"{_PARAM_DIR}/IsOffroad") as f:
      return f.read().strip() == "1"
  except OSError:
    # 读失败按 offroad 处理: 失败方向偏向退出省电, 宁可误杀不可漏杀
    return True


def offroad_watchdog(monotonic=None, sleep=None, exit_fn=None) -> None:
  """offroad 累计超过宽限期则自动退出。monotonic/sleep/exit_fn 可注入, 供 PC 端单测。"""
  monotonic = monotonic or time.monotonic
  sleep = sleep or time.sleep
  exit_fn = exit_fn or _self_terminate
  offroad_since = None  # None=onroad 中; 时间戳=当前这轮 offroad 的起点
  while True:
    sleep(OFFROAD_PARAMS_POLL_SEC)
    now = monotonic()
    if _read_is_offroad():
      if offroad_since is None:
        offroad_since = now
        print(f"[offroad_wd] 进入 offroad, {OFFROAD_GRACE_SEC}s 后自动退出", flush=True)
      if now - offroad_since >= OFFROAD_GRACE_SEC:
        print(f"[offroad_wd] offroad 已 {now - offroad_since:.0f}s, 自动退出 DLNA", flush=True)
        exit_fn()
        return
    else:
      if offroad_since is not None:
        print("[offroad_wd] 重新 onroad, 取消退出倒计时", flush=True)
        offroad_since = None


def main():
  # manager 启动时 stdout 被重定向到 /dev/null, 自建日志文件
  class Tee:
    def __init__(s, path):
      s.f = open(path, "a", buffering=1)
      s.orig = sys.__stdout__
    def write(s, m):
      s.f.write(m)
      try: s.orig.write(m)
      except Exception: pass
    def flush(s):
      s.f.flush()
  sys.stdout = sys.stderr = Tee("/data/dlna_live.log")
  print(f"\n===== dlna_live 启动 {time.strftime('%F %T')} =====", flush=True)

  # 1. 打开官方直播开关(拉起 camerad + stream_encoderd)
  try:
    _write_param("IsLiveStreaming", "1")
    print("[main] IsLiveStreaming=1", flush=True)
  except OSError as e:
    print(f"[main] 写 IsLiveStreaming 失败: {e}", flush=True)

  ip_holder = [local_ip()]
  print(f"[main] 本机 IP: {ip_holder[0]}", flush=True)

  Handler.adv_ip = ip_holder[0]

  threading.Thread(target=ssdp_responder, args=(ip_holder,), daemon=True).start()
  threading.Thread(target=ssdp_alive, args=(ip_holder,), daemon=True).start()

  # 定期刷新 adv_ip(网络切换时) + 重申 IsLiveStreaming(防被 teardown 误清)
  def ip_refresher():
    last_ip = ip_holder[0]
    while True:
      time.sleep(20)
      if _EXITING.is_set():
        break
      ip = local_ip()
      if ip != last_ip:
        print(f"[main] 本机 IP 变更: {last_ip} → {ip}", flush=True)
        last_ip = ip
      ip_holder[0] = ip
      Handler.adv_ip = ip
      try:
        if not _EXITING.is_set():
          _write_param("IsLiveStreaming", "1")
      except OSError:
        pass
  threading.Thread(target=ip_refresher, daemon=True).start()

  # 2. 退出路径统一走 cleanup_live_stream (复位 IsLiveStreaming)
  #    SIGTERM (manager 杀进程 / offroad 看门狗) + SIGINT (调试 Ctrl-C) + atexit + finally 四路
  atexit.register(cleanup_live_stream)
  signal.signal(signal.SIGTERM, lambda *_: (cleanup_live_stream(), sys.exit(0)))
  signal.signal(signal.SIGINT, lambda *_: (cleanup_live_stream(), sys.exit(0)))

  # 3. offroad 看门狗: 熄火后宽限 60s 自动退出, 同时把 DlnaLiveEnabled 总开关复位为 0
  threading.Thread(target=offroad_watchdog, daemon=True, name="offroad_watchdog").start()

  srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
  srv.daemon_threads = True
  print(f"[main] HTTP+DLNA 服务 :{HTTP_PORT} 就绪", flush=True)
  try:
    srv.serve_forever()
  finally:
    cleanup_live_stream()


if __name__ == "__main__":
  main()
