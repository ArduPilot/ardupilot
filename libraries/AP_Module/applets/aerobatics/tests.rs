// SPDX-License-Identifier: GPL-3.0-or-later
extern crate alloc;
mod math;
mod primitives;

#[cfg(test)]
mod tests {
    use super::primitives::Roll;
    #[test]
    fn roll_edges() {
        assert!(Roll::sequence(&[]).is_err());
        let one = Roll::sequence(&[(1.0, 180.0)]).unwrap();
        assert_eq!(one.get(0.5, 1.0, 60.0), 90.0);
        assert_eq!(one.get(2.0, 1.0, 60.0), 180.0);
        assert!(Roll::Entry(0.0).get(-0.1, 1.0, 60.0).is_nan());
        assert_eq!(Roll::EntryExit(90.0).get(0.25, 0.0, 60.0), 45.0);
        assert_eq!(Roll::EntryExit(90.0).get(0.25, 1.0, 0.0), 45.0);
        assert_eq!(
            Roll::Entry(90.0).get(0.1, 10.0, 30.0),
            90.0_f32 * 0.1 / ((90.0 / 30.0) / 10.0)
        );
        assert!(Roll::Entry(f32::NAN).get(0.0, 1.0, 60.0).is_nan());
        // The original sequence divides by total=0: NaN end-time comparisons
        // fall through to the last angle. Schedule validation must reject it.
        let zero = Roll::sequence(&[(0.0, 180.0)]).unwrap();
        assert_eq!(zero.get(0.5, 1.0, 60.0), 180.0);
    }
}
