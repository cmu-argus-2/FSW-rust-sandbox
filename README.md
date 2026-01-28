## Argus Notes
This repo is based off the <a href="https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples"><strong>rp-235x-hal-examples</a> folder in the <a href="https://github.com/rp-rs/rp-hal">rp-hal</a> repo.
If there's anything confusing about the general rust setup outside of these notes, try checking them out.

## Getting Started

To build the examples, first grab a copy of the source code:

```console
$ git clone https://github.com/cmu-argus-2/FSW-rust-sandbox.git
```

Then use `rustup` to grab the Rust Standard Library for the appropriate targets.
RP2350 has two possible targets: `thumbv8m.main-none-eabihf` for the Arm mode, and
`riscv32imac-unknown-none-elf` for the RISC-V mode.

```console
$ rustup target add thumbv8m.main-none-eabihf
$ rustup target add riscv32imac-unknown-none-elf
```


Next, install and build picotool. This can be found in Appendix B of <a href="https://pip-assets.raspberrypi.com/categories/610-raspberry-pi-pico/documents/RP-008276-DS-1-getting-started-with-pico.pdf?disposition=inline">Getting started with Raspberry Pi Pico-series</a>.

Note: An issue when building picotool might be that you're missing the pico-sdk, so go to <a href="https://github.com/raspberrypi/pico-sdk">Raspberry Pi Pico SDK</a> and put it in the file ~/pico/pico-sdk (any path that matches the export in the building section).

Note: On MacOS during the "Building picotool" step, if it says "exception file not found", try this command instead of the cmake command:

```
rm -rf build cmake -B build \ -DCMAKE_C_COMPILER=clang \ -DCMAKE_CXX_COMPILER=clang++ \ -DCMAKE_OSX_SYSROOT=$(xcrun --show-sdk-path) cmake --build build
```
It forces cmake to use clang and clang++ instead of other compilers and tells it the path to the MacOS SDK.

Then connect a usb cable to the Mainboard and set it to Boot mode (reset while pressing Boot). To flash a program onto the board and run it, do
```
cargo run --bin blinky
```
Replace "blinky" with any other filename in src/bin to run that file on the board.

To build without running, do 
```
cargo build --bin blinky
```
It will show up in `./target/thumbv8m.main-none-eabihf/debug/blinky`.

For the release profile build pass `--release` to the `cargo build`
or `cargo run` commands. This will build the example with optimizations enabled:

```console
$ cargo run --target thumbv8m.main-none-eabihf --release --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --release --bin blinky
```

the cargo commands (run/build) will default to `thumbv8m.main-none-eabihf` (Cortex-M33) if no target is specified.

