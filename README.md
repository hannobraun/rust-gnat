# Tlera Corp Gnat Board Support Crate

## About

This is an unofficial Board Support Crate (BSC) for using the [Tlera Corp Gnat] LoRa+GNSS Asset Tracker with the [Rust] programming language.

It is currently in its early stages. Documentation is sparse and features are basic. Pull requests welcome!

[Tlera Corp Gnat]: https://www.tindie.com/products/TleraCorp/gnat-loragnss-asset-tracker/
[Rust]: https://www.rust-lang.org/


## Usage

Currently, two ways for flashing code to the Gnat are supported:
- Using the built-in bootloader via [dfu-util].
- Using an external STLINK programmer via [OpenOCD].

Open `.cargo/config` and uncomment the runner that matches your preferred configuration (comment all other ones). Then you can flash an example program like this:

```
cargo run --example led
```

[dfu-util]: http://dfu-util.sourceforge.net/
[OpenOCD]: http://openocd.org/


## License

This project is open source software, licensed under the terms of the [Zero Clause BSD License][] (0BSD, for short). This basically means you can do anything with the software, without any restrictions, but you can't hold the authors liable for problems.

See [LICENSE.md] for full details.

[Zero Clause BSD License]: https://opensource.org/licenses/0BSD
[LICENSE.md]: https://github.com/braun-embedded/rust-gnat/blob/master/LICENSE.md
