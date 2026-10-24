# Binding generator metadata

The Python parser can export its parsed binding metadata for other binding
backends without generating Lua code:

```sh
python3 libraries/AP_Scripting/generator/generator.py \
    -i libraries/AP_Scripting/generator/description/bindings.desc \
    --dump-ir /tmp/ap-binding-ir.json
python3 -m pytest -q libraries/AP_Scripting/generator/tests/test_ir.py
```

`--dump-ir` may also accompany the normal `-o`/`-d` options. Schema 1 exports
includes, singleton access/locking metadata, methods, and aliases. The additive
`userdata` section also exports creation metadata, fields (including access,
ranges, arrays, validity masks and dependencies), and operator tokens. `operator_names` lists
the generated operators; `operators_manual` marks additional manual operators.
The older `operations` integer remains a presence marker, not a stable bit layout.
The additive `ap_objects` section uses the same shape for pointer-backed objects.
The export does not establish who owns them, their lifetime or cleanup rules;
native consumers must supply those policies before exposing handles.
Singleton fields, enum values and manual implementation bodies are not exported.
Consumers must reject unsupported contracts instead of inferring missing
semantics. Array lengths, range bounds and validity masks retain opaque C++
expressions; consumers must audit them, not evaluate them as Python expressions.

Userdata arguments do not describe C++ constness or mutation direction: Lua
passes userdata by reference even without a `'Ref` annotation. Native backends
must audit the underlying C++ methods before defining value-copy contracts.

Types distinguish `nullable` (`'Null`), `reference` (`'Ref`), and `pointer`
(the leading `&`, which makes the generated call pass the output's address).
The `name` identifies a userdata, borrowed object, enum type or literal value.
A null range means no explicit bounds. Lua clamps explicit integer bounds to
the type's limits and uses inclusive comparisons. Its numeric conversion and
finite-number rules also live in `emit_checker` and the Lua numeric helpers;
the IR does not implement or replace those checks. Native backends must audit
their own numeric encodings and validation.

Lua evaluates arguments and range expressions before taking the binding's
semaphore. A native backend may deliberately move checks inside the lock to
avoid a check/use gap, but must document that difference. Lock flags reflect
the parser state used by the Lua emitter, including existing parser behaviour.

Lists follow internal parser order, not sorted declaration order. Fields are
in reverse declaration order; operators follow the emitter's fixed bit order.
Consumers must use an audited component order for wire layouts and sort for
deterministic IDs. An alias with `manual: false` refers to a
parsed method; `manual: true` requires a separate implementation. Conditional
includes, owners, methods and aliases retain their unpreprocessed expressions.
No target configuration is evaluated during export.
