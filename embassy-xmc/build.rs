//! cortex-m-rt (Feature `device`) bindet im Linker-Skript `device.x` ein. Die Vektortabelle
//! mit den Interrupt-Einträgen (`__INTERRUPTS`) steht in `src/interrupt.rs`, deshalb bleibt
//! `device.x` leer. Wird später das PAC `xmc4700` eingebunden, liefert dieses beides.

use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(
        out.join("device.x"),
        "/* leer: __INTERRUPTS kommt aus embassy-xmc/src/interrupt.rs */\n",
    )
    .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
