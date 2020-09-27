Simple logic analyzer written in embedded rust.

## Building

Install rust from [rustup.rs](http://rustup.rs). Then add the target for
the correct architecture:

```
rustup target add thumbv7m-none-eabi
```

After that, you can build with:

```
cargo build --release
```


## Hardware

Currently this software runs only on stm32f103xx devices, like the
blue/black pill boards, which you can find on the sites of various
online retailers.

## Presentation

[Here](./presentation.pdf) is the presentation.