// SPDX-License-Identifier: GPL-3.0-or-later
//! Dynamic parameter tables declared in one place, like a Lua applet's
//! bind_add_param() calls: each entry keeps its storage index, name, type,
//! default and `// @Param:` documentation together. See `param_table!`.

use crate::{Context, Error, Parameter, Result};

/// Longest full parameter name, prefix included.
pub const PARAM_NAME_MAX: usize = 16;

#[derive(Clone, Copy, Debug)]
pub struct ParamDef {
    /// Storage index within the table, 1 to 63. Never reuse or renumber.
    pub index: u8,
    /// Name without the table prefix.
    pub name: &'static str,
    pub default: f32,
}

/// Compile-time table checks; returns the table size to register.
pub const fn check_param_table(prefix: &str, defs: &[ParamDef]) -> u8 {
    assert!(defs.len() <= 63, "at most 63 parameters per table");
    let mut size = 0;
    let mut i = 0;
    while i < defs.len() {
        let d = &defs[i];
        assert!(
            d.index >= 1 && d.index <= 63,
            "parameter index must be 1 to 63"
        );
        assert!(!d.name.is_empty(), "empty parameter name");
        assert!(
            prefix.len() + d.name.len() <= PARAM_NAME_MAX,
            "parameter name longer than 16 characters"
        );
        let mut j = i + 1;
        while j < defs.len() {
            assert!(defs[j].index != d.index, "duplicate parameter index");
            assert!(!same_name(defs[j].name, d.name), "duplicate parameter name");
            j += 1;
        }
        if d.index >= size {
            size = d.index;
        }
        i += 1;
    }
    size
}

const fn same_name(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

/// Retry calls which contend with Lua or another native module.
fn retry<T>(c: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = c.time_us();
    loop {
        match call() {
            Err(Error::Busy) if c.time_us().saturating_sub(start) < 100_000 => c.sleep_ms(1)?,
            result => return result,
        }
    }
}

impl ParamDef {
    pub fn register(&self, c: &Context<'_>, key: u8) -> Result {
        let ok = retry(c, || {
            c.param()
                .add_param(key, self.index, self.name, self.default)
        })?;
        if ok {
            Ok(())
        } else {
            Err(Error::Host)
        }
    }

    /// Bind a live handle, preserving updates made by the ground station.
    pub fn bind<'a, 'h>(&self, c: &'a Context<'h>, prefix: &str) -> Result<Parameter<'a, 'h>> {
        let mut buf = [0; PARAM_NAME_MAX];
        let name = full_name(prefix, self.name, &mut buf)?;
        retry(c, || c.parameter(name))?.ok_or(Error::Invalid)
    }
}

fn full_name<'b>(prefix: &str, name: &str, buf: &'b mut [u8; PARAM_NAME_MAX]) -> Result<&'b str> {
    let n = prefix.len() + name.len();
    if n > buf.len() {
        return Err(Error::Invalid);
    }
    buf[..prefix.len()].copy_from_slice(prefix.as_bytes());
    buf[prefix.len()..n].copy_from_slice(name.as_bytes());
    core::str::from_utf8(&buf[..n]).map_err(|_| Error::Invalid)
}

#[doc(hidden)]
pub fn register_param_table(c: &Context<'_>, key: u8, prefix: &str, size: u8) -> Result {
    if retry(c, || c.param().add_table(key, prefix, size))? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

/// Registered table with opt-in cached values.
pub struct ParamTable<'a, 'h, const N: usize> {
    handles: [Parameter<'a, 'h>; N],
    cache: [f32; N],
}

impl<'a, 'h, const N: usize> ParamTable<'a, 'h, N> {
    pub fn register(
        c: &'a Context<'h>,
        key: u8,
        prefix: &'static str,
        size: u8,
        defs: &'static [ParamDef; N],
    ) -> Result<Self> {
        register_param_table(c, key, prefix, size)?;
        let mut handles: [Option<Parameter<'a, 'h>>; N] = core::array::from_fn(|_| None);
        let mut cache = [0.0; N];
        for (i, d) in defs.iter().enumerate() {
            d.register(c, key)?;
            let p = d.bind(c, prefix)?;
            cache[i] = p.get()?.ok_or(Error::Invalid)?;
            handles[i] = Some(p);
        }
        Ok(Self {
            handles: handles.map(Option::unwrap),
            cache,
        })
    }

    /// Re-read all values; bits refer to declaration positions, not storage indices.
    /// On error, retain the cache so a retry still reports every changed entry.
    pub fn refresh(&mut self) -> Result<u64> {
        let mut values = self.cache;
        let mut changed = 0;
        for (i, p) in self.handles.iter().enumerate() {
            let v = p.get()?.ok_or(Error::Invalid)?;
            if v != self.cache[i] {
                values[i] = v;
                changed |= 1 << i;
            }
        }
        self.cache = values;
        Ok(changed)
    }

    pub fn get(&self, i: usize) -> f32 {
        self.cache[i]
    }

    fn write(&mut self, i: usize, v: f32, save: bool) -> Result {
        let ok = if save {
            self.handles[i].set_and_save(v)?
        } else {
            self.handles[i].set(v)?
        };
        if !ok {
            return Err(Error::Host);
        }
        self.cache[i] = v;
        Ok(())
    }

    pub fn set(&mut self, i: usize, v: f32) -> Result {
        self.write(i, v, false)
    }
    pub fn set_and_save(&mut self, i: usize, v: f32) -> Result {
        self.write(i, v, true)
    }
}

/// Declare a dynamic parameter table. Entries are
/// `Variant = index, "NAME", type: default;` with type `f32`, each preceded by `// @Param: PREFIX_NAME` documentation
/// in the format param_parse.py reads from C++ and Lua:
///
/// ```rust
/// ap_rust_sdk::param_table! {
///     pub struct Params, enum P;
///     key = 112, prefix = "SHOW_";
///
///     // @Param: SHOW_TAKEOFF_ALT
///     // @DisplayName: Takeoff altitude
///     // @Description: Altitude above the current position to take off to
///     // @Units: m
///     // @User: Standard
///     TakeoffAlt = 17, "TAKEOFF_ALT", f32: 2.5;
/// }
/// ```
///
/// Keys must be in the host binding's range, 0 to 200, even if registration
/// is not called:
///
/// ```compile_fail
/// ap_rust_sdk::param_table! {
///     struct Invalid, enum P;
///     key = 201, prefix = "BAD_";
///     Value = 1, "VALUE", f32: 0.0;
/// }
/// ```
///
/// ```compile_fail
/// ap_rust_sdk::param_table! {
///     struct Invalid, enum P;
///     key = 201, prefix = "BAD_";
///     size = 63;
///     Value = 1, "VALUE", f32: 0.0;
/// }
/// ```
///
/// This generates `enum P` naming the entries, a `Params` struct with
/// `register(c)`, `refresh()` and typed get/set methods taking `P`, and
/// `P::changed(mask)` for testing a `refresh()` result. Names, indices and
/// table sizes are checked at compile time. Optional `size = N;` preserves
/// reserved slots in existing tables. Use `register_definitions()` and `bind()`
/// for live handles, or `register_table()` and individual definitions' `register()`
/// for parameters created conditionally. `register()` opts into cached values.
#[macro_export]
macro_rules! param_table {
    ($vis:vis struct $table:ident, enum $id:ident;
     key = $key:expr, prefix = $prefix:expr; size = $size:expr; $($entries:tt)+) => {
        $crate::param_table!(@table $vis struct $table, enum $id;
                            key = $key, prefix = $prefix; [$size] $($entries)+);
    };
    ($vis:vis struct $table:ident, enum $id:ident;
     key = $key:expr, prefix = $prefix:expr;
     $( $variant:ident = $index:expr, $name:literal, f32: $default:expr; )+) => {
        $crate::param_table!(@table $vis struct $table, enum $id;
                            key = $key, prefix = $prefix; []
                            $( $variant = $index, $name, f32: $default; )+);
    };
    (@table
        $vis:vis struct $table:ident, enum $id:ident;
        key = $key:expr, prefix = $prefix:expr;
        [$($size:expr)?]
        $( $variant:ident = $index:expr, $name:literal, f32: $default:expr; )+
    ) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr(usize)]
        $vis enum $id { $( $variant ),+ }

        impl $id {
            /// True if this entry is set in a `refresh()` mask.
            #[allow(dead_code)]
            $vis fn changed(self, mask: u64) -> bool {
                mask & (1 << self as usize) != 0
            }
        }

        $vis struct $table<'a, 'h>(
            $crate::ParamTable<'a, 'h, { $crate::param_table!(@count $($variant)+) }>,
        );

        // Run the key and layout checks even if registration is unused.
        const _: u8 = {
            assert!($table::<'static, 'static>::KEY <= 200, "parameter table key must be 0 to 200");
            $table::<'static, 'static>::SIZE
        };

        #[allow(dead_code)]
        impl<'a, 'h> $table<'a, 'h> {
            const DEFS: [$crate::ParamDef; $crate::param_table!(@count $($variant)+)] = [
                $( $crate::ParamDef { index: $index, name: $name,
                                      default: $default as f32 } ),+
            ];
            $vis const KEY: u8 = $key;
            $vis const PREFIX: &'static str = $prefix;
            $vis const SIZE: u8 = {
                let minimum = $crate::check_param_table($prefix, &Self::DEFS);
                let size = $crate::param_table!(@size minimum $(, $size)?);
                assert!(size >= minimum && size <= 63, "invalid parameter table size");
                size
            };
            $vis fn definitions() -> &'static [$crate::ParamDef] { &Self::DEFS }
            $vis const fn definition(p: $id) -> $crate::ParamDef { Self::DEFS[p as usize] }
            $vis fn register_table(c: &$crate::Context<'_>) -> $crate::Result {
                $crate::register_param_table(c, Self::KEY, Self::PREFIX, Self::SIZE)
            }
            /// Register definitions without caching values or allocating handles.
            $vis fn register_definitions(c: &$crate::Context<'_>) -> $crate::Result {
                Self::register_table(c)?;
                for d in &Self::DEFS { d.register(c, Self::KEY)?; }
                Ok(())
            }
            $vis fn bind(c: &'a $crate::Context<'h>, p: $id) -> $crate::Result<$crate::Parameter<'a, 'h>> {
                Self::definition(p).bind(c, Self::PREFIX)
            }

            $vis fn register(c: &'a $crate::Context<'h>) -> $crate::Result<Self> {
                Ok(Self($crate::ParamTable::register(c, Self::KEY, Self::PREFIX, Self::SIZE, &Self::DEFS)?))
            }
            /// Re-read all values; test the result with `changed()`.
            $vis fn refresh(&mut self) -> $crate::Result<u64> { self.0.refresh() }
            $vis fn get(&self, p: $id) -> f32 { self.0.get(p as usize) }
            $vis fn set(&mut self, p: $id, v: f32) -> $crate::Result { self.0.set(p as usize, v) }
            $vis fn set_and_save(&mut self, p: $id, v: f32) -> $crate::Result { self.0.set_and_save(p as usize, v) }

        }
    };
    (@size $minimum:ident) => { $minimum };
    (@size $minimum:ident, $size:expr) => { $size };
    (@count) => { 0usize };
    (@count $head:ident $($tail:ident)*) => { 1usize + $crate::param_table!(@count $($tail)*) };
}
