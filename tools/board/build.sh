#!/usr/bin/env bash
set -euo pipefail

: "${TI_COMPILER_ROOT:?Set TI_COMPILER_ROOT to the TI ARM Clang installation}"
: "${MSPM0_SDK_ROOT:?Set MSPM0_SDK_ROOT to the MSPM0 SDK installation}"
usage='Usage: bash tools/board/build.sh 3|4|5 [sensor|display]'
role=""
case "${1:-}" in
  3) lab=lab3_spi ;;
  4) lab=lab4_i2c ;;
  5)
    lab=lab5_adc_uart_crc
    case "${2:-}" in
      sensor | display) role=$2 ;;
      *) echo "$usage (lab 5 needs a role)" >&2; exit 2 ;;
    esac
    ;;
  *) echo "$usage" >&2; exit 2 ;;
esac

board_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$board_dir/../.." && pwd)
output_dir="$repo_dir/$lab/Debug/uart"
image="$lab${role:+_$role}"
mkdir -p "$output_dir"
compiler="$TI_COMPILER_ROOT/bin/tiarmclang"
sdk_source="$MSPM0_SDK_ROOT/source"
flags=(-mcpu=cortex-m0plus -mthumb -O2 -g -D__MSPM0G3507__
       -I"$board_dir" -isystem "$sdk_source"
       -isystem "$sdk_source/third_party/CMSIS/Core/Include")
objects=()

if [[ $lab == lab5_adc_uart_crc ]]; then
  # FreeRTOS kernel, built the same way as the freertos_kernel CCS project
  # (heap_4, TI ARM Clang CM0 port, TI's static-allocation hooks).
  kernel="$MSPM0_SDK_ROOT/kernel/freertos"
  flags+=(-I"$repo_dir/freertos_kernel"
          -isystem "$kernel/Source/include"
          -isystem "$kernel/Source/portable/TI_ARM_CLANG/ARM_CM0"
          -isystem "$sdk_source/ti/posix/ticlang"
          -DLAB5_DEVICE_ROLE_${role^^})
  kernel_dir="$output_dir/freertos"
  mkdir -p "$kernel_dir"
  for source in \
    "$kernel/Source/tasks.c" "$kernel/Source/queue.c" \
    "$kernel/Source/list.c" "$kernel/Source/timers.c" \
    "$kernel/Source/event_groups.c" "$kernel/Source/stream_buffer.c" \
    "$kernel/Source/portable/MemMang/heap_4.c" \
    "$kernel/Source/portable/TI_ARM_CLANG/ARM_CM0/port.c" \
    "$kernel/Source/portable/TI_ARM_CLANG/ARM_CM0/portasm.c" \
    "$kernel/dpl/AppHooks_freertos.c" "$kernel/dpl/StaticAllocs_freertos.c" \
    "$sdk_source/ti/posix/freertos/PTLS.c"; do
    object="$kernel_dir/$(basename "$source" .c).o"
    "$compiler" "${flags[@]}" -std=c11 -w -c "$source" -o "$object"
    objects+=("$object")
  done
fi

"$compiler" "${flags[@]}" -std=c++17 -Wall -Wextra -Werror -c \
  "$repo_dir/$lab/$lab.cpp" -o "$output_dir/$image.o"
"$compiler" "${flags[@]}" -std=c11 -Wall -Wextra -Werror -c \
  "$board_dir/board_runtime.c" -o "$output_dir/runtime.o"
"$compiler" "${flags[@]}" -c \
  "$sdk_source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
  -o "$output_dir/startup.o"
"$compiler" -mcpu=cortex-m0plus -mthumb -o "$output_dir/$image.out" \
  "$output_dir/startup.o" "$output_dir/runtime.o" "$output_dir/$image.o" \
  "${objects[@]}" \
  -Wl,-l,"$sdk_source/ti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a" \
  -Wl,--rom_model -Wl,-l,libc.a -Wl,-m,"$output_dir/$image.map" \
  "$board_dir/memory.cmd"
"$TI_COMPILER_ROOT/bin/tiarmobjcopy" -O binary \
  "$output_dir/$image.out" "$output_dir/$image.bin"
echo "Built $output_dir/$image.out"
