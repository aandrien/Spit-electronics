#!/bin/bash
# Harden a Raspberry Pi against SD-card corruption and bricking from frequent
# power cuts. Apply once per Pi. Safe to re-run — every step is idempotent.
#
# What it does:
#   [1/4] Mount /boot read-only — prevents brick-causing corruption of the
#         FAT32 boot partition (the partition that holds the bootloader).
#   [2/4] Install log2ram — moves /var/log to RAM, syncs to disk daily.
#         Cuts steady-state SD wear from journald/syslog.
#   [3/4] Disable swap — eliminates a write source and corruption surface.
#         The Pi 4 has plenty of RAM for the Spit workload.
#   [4/4] Disable unattended apt timers — stops apt from writing in the
#         background at random times. You still control manual apt runs.
#
# After running, reboot to fully activate log2ram + the ro /boot mount.
#
# When you need to update kernel/firmware or edit /boot/config.txt:
#   sudo mount -o remount,rw /boot         (or /boot/firmware on Bookworm)
#   # ... do your edits ...
#   sudo mount -o remount,ro /boot
#
# To undo any single step, restore the fstab backup this script makes, or
# reinstall the removed packages — nothing here is irreversible.
#
# Usage:
#   ./RPi/bash_scripts/harden_pi.sh

set -euo pipefail

if [ "$EUID" -eq 0 ]; then
  echo "!! Don't run this as root. Run as the 'pi' user; the script sudos where needed."
  exit 1
fi

echo "==> Pi power-loss hardening"
echo

# -------------------------------------------------------------------
echo "==> [1/4] Mount /boot read-only"
# -------------------------------------------------------------------
FSTAB=/etc/fstab
# Pi OS uses /boot on Bullseye and earlier, /boot/firmware on Bookworm+.
BOOT_MOUNT=$(awk '$2 == "/boot" || $2 == "/boot/firmware" { print $2; exit }' "$FSTAB")
if [ -z "$BOOT_MOUNT" ]; then
  echo "   !! Couldn't find a /boot line in $FSTAB. Skipping."
else
  if awk -v m="$BOOT_MOUNT" '$2 == m && $4 ~ /(^|,)ro(,|$)/ { found=1 } END { exit !found }' "$FSTAB"; then
    echo "   $BOOT_MOUNT already mounted ro in fstab."
  else
    BAK="$FSTAB.bak.$(date +%s)"
    sudo cp "$FSTAB" "$BAK"
    echo "   backed up $FSTAB to $BAK"
    sudo awk -v m="$BOOT_MOUNT" '
      $2 == m && $4 !~ /(^|,)ro(,|$)/ { $4 = "ro," $4 }
      { print }
    ' "$FSTAB" | sudo tee /tmp/fstab.new >/dev/null
    sudo mv /tmp/fstab.new "$FSTAB"
    echo "   added ro to the $BOOT_MOUNT line"
  fi
  if findmnt -n -o OPTIONS "$BOOT_MOUNT" 2>/dev/null | grep -qw ro; then
    echo "   $BOOT_MOUNT currently mounted ro."
  else
    echo "   remounting $BOOT_MOUNT ro now..."
    if ! sudo mount -o remount "$BOOT_MOUNT"; then
      echo "   !! remount failed — will apply on next reboot."
    fi
  fi
fi
echo

# -------------------------------------------------------------------
echo "==> [2/4] Install log2ram"
# -------------------------------------------------------------------
if command -v log2ram >/dev/null 2>&1 || [ -f /etc/log2ram.conf ]; then
  echo "   log2ram already installed."
else
  echo "   adding azlux repo..."
  sudo wget -qO /usr/share/keyrings/azlux-archive-keyring.gpg https://azlux.fr/repo.gpg
  echo "deb [signed-by=/usr/share/keyrings/azlux-archive-keyring.gpg] http://packages.azlux.fr/debian/ stable main" \
    | sudo tee /etc/apt/sources.list.d/azlux.list >/dev/null
  sudo apt update
  sudo apt install -y log2ram
  echo "   log2ram installed. Active after next reboot."
fi
echo

# -------------------------------------------------------------------
echo "==> [3/4] Disable swap"
# -------------------------------------------------------------------
if dpkg -s dphys-swapfile >/dev/null 2>&1; then
  sudo dphys-swapfile swapoff || true
  sudo systemctl disable --now dphys-swapfile || true
  sudo apt remove --purge -y dphys-swapfile
  echo "   removed dphys-swapfile."
else
  echo "   dphys-swapfile not installed."
fi
if [ -n "$(swapon --show)" ]; then
  echo "   !! swap still active — investigate manually:"
  swapon --show
else
  echo "   no swap active."
fi
echo

# -------------------------------------------------------------------
echo "==> [4/4] Disable unattended apt timers"
# -------------------------------------------------------------------
for t in apt-daily.timer apt-daily-upgrade.timer; do
  if systemctl is-enabled --quiet "$t" 2>/dev/null; then
    sudo systemctl disable --now "$t"
    sudo systemctl mask "$t"
    echo "   disabled and masked $t"
  else
    echo "   $t already disabled"
  fi
done
echo

echo "==> Done. Reboot to fully activate log2ram and the ro /boot mount:"
echo "     sudo reboot"
echo
echo "When you need to update kernel/firmware or edit $BOOT_MOUNT/config.txt:"
echo "     sudo mount -o remount,rw $BOOT_MOUNT"
echo "     # ... do your edits ..."
echo "     sudo mount -o remount,ro $BOOT_MOUNT"
