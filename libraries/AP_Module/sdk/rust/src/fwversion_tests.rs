// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{Error, FirmwareString};

#[test]
fn firmware_string_validates_wire() {
    let mut wire = [0; 65];
    wire[0] = 8;
    wire[1..9].copy_from_slice(b"1234abcd");
    let value = FirmwareString::decode(&wire).unwrap();
    assert_eq!(value.as_bytes(), b"1234abcd");
    assert_eq!(value.as_str(), "1234abcd");
    assert_eq!(value.len(), 8);
    assert!(!value.is_empty());

    wire[0] = 65;
    assert_eq!(FirmwareString::decode(&wire), Err(Error::Host));
    wire = [0; 65];
    wire[1] = 0xff;
    wire[0] = 1;
    assert_eq!(FirmwareString::decode(&wire), Err(Error::Host));
    wire = [0; 65];
    wire[2] = 1;
    assert_eq!(FirmwareString::decode(&wire), Err(Error::Host));
    assert_eq!(FirmwareString::decode(&wire[..64]), Err(Error::Host));
}
