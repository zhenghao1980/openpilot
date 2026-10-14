#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="$(cat /VERSION)"
fi

export STAGING_ROOT="/data/safe_staging"

# 2026-10-11: 1GB zram swap to relieve memory pressure (tizi 3.6GB RAM)
# 单块 1G zram0（lzo：本内核 zram 只支持 lzo/deflate，hot_add 在新内核被禁，
# 双设备方案不可靠；swapoff→reset→扩容可在运行时安全重建）
if ! grep -q zram0 /proc/swaps 2>/dev/null; then
  sudo -n sh -c "echo lzo > /sys/block/zram0/comp_algorithm 2>/dev/null; echo 1G > /sys/block/zram0/disksize && mkswap /dev/zram0 >/dev/null && swapon -p 100 /dev/zram0" 2>/dev/null || true
fi
# swappiness=100: swap early to zram instead of late burst; page-cluster=0: swap in 1 page at a time (no latency spikes)
# min_free_kbytes=128MB: kswapd keeps real headroom so UI never stalls in direct reclaim
sudo -n sysctl -w vm.swappiness=100 vm.page-cluster=0 vm.min_free_kbytes=131072 vm.vfs_cache_pressure=150 >/dev/null 2>&1 || true
