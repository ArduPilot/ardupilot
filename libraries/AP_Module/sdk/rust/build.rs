// SPDX-License-Identifier: GPL-3.0-or-later

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let root = manifest.parent().unwrap().parent().unwrap();
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("AP_Module");
    let python = env::var_os("PYTHON").unwrap_or_else(|| "python3".into());
    let status = Command::new(python)
        .arg(root.join("generator/generate.py"))
        .arg("--output-dir")
        .arg(&output)
        .status()
        .expect("failed to run AP_Module binding generator");
    assert!(status.success(), "AP_Module binding generator failed");
    println!("cargo:rerun-if-changed={}", root.join("generator").display());
    println!("cargo:rerun-if-changed={}", manifest.join("src/compatibility.rs").display());
    println!("cargo:rerun-if-changed={}", root.join("../AP_Scripting/generator").display());
    println!("cargo:rustc-env=AP_MODULE_SDK_GENERATED={}", output.join("sdk/rust/src").display());
}
