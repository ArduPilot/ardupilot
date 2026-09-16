#!/usr/bin/env python3

"""
Unit tests for enum_parse.py

AP_FLAKE8_CLEAN
"""

import os
import tempfile
import unittest

from enum_parse import EnumDocco


class TestMatchEnumLine(unittest.TestCase):
    """Tests for the line-matching of individual enumeration entries."""

    def setUp(self):
        self.docco = EnumDocco(None)

    def assert_match(self, line, expected):
        self.assertEqual(self.docco.match_enum_line(line), expected)

    def test_bare_entries(self):
        self.assert_match("    FRED,", ("FRED", None, None))
        self.assert_match("    FRED,  // a comment", ("FRED", None, "a comment"))

    def test_decimal_values(self):
        self.assert_match("    FRED = 17,", ("FRED", "17", None))
        self.assert_match("    FRED = -1,  // a comment", ("FRED", "-1", "a comment"))
        self.assert_match("    FRED = 0,", ("FRED", "0", None))
        self.assert_match("    FRED = -1L,", ("FRED", "-1", None))

    def test_hexadecimal_values(self):
        self.assert_match("    FRED = 0x1A,", ("FRED", 26, None))
        self.assert_match("    FRED = 0X1a,  // a comment", ("FRED", 26, "a comment"))
        self.assert_match("    FRED = 0x0000000000000001ULL,", ("FRED", 1, None))

    def test_shifted_values(self):
        self.assert_match("    FRED = 1U<<3,", ("FRED", 8, None))
        self.assert_match("    FRED = (1 << 3),", ("FRED", 8, None))
        self.assert_match("    FRED = (1U << 0U),", ("FRED", 1, None))
        self.assert_match("    FRED = (1U << 3),  // a comment", ("FRED", 8, "a comment"))
        self.assert_match("    FRED = (3U << 6),", ("FRED", 192, None))
        self.assert_match("    FRED = 0 << 2,", ("FRED", 0, None))
        self.assert_match("    FRED = (1U << 31),", ("FRED", 1 << 31, None))
        self.assert_match("    FRED = (1ULL << 40),", ("FRED", 1 << 40, None))
        self.assert_match("    FRED = (1ULL << 63),", ("FRED", 1 << 63, None))

    def test_integer_suffixes(self):
        # all legal C++ integer suffixes; an unrecognised one used to drop the enumeration
        for suffix in ["", "u", "U", "l", "L", "ll", "LL", "ul", "UL", "lu", "LU", "ull", "ULL", "llu", "LLU", "uLL", "LLu"]:
            with self.subTest(suffix=suffix):
                self.assert_match(f"    FRED = 17{suffix},", ("FRED", "17", None))
                self.assert_match(f"    FRED = 0x1A{suffix},", ("FRED", 26, None))
                self.assert_match(f"    FRED = (1{suffix} << 3{suffix}),", ("FRED", 8, None))

    def test_comment_without_trailing_comma(self):
        # the last entry of an enumeration often has no comma
        self.assert_match("    FRED = 0x1A  // a comment", ("FRED", 26, "a comment"))
        self.assert_match("    FRED = (1 << 3)  // a comment", ("FRED", 8, "a comment"))

    def test_doxygen_comment_markers_are_removed(self):
        for marker in ["//", "///", "///<", "//!<", "//<"]:
            with self.subTest(marker=marker):
                self.assert_match(f"    FRED,  {marker} a comment", ("FRED", None, "a comment"))
                self.assert_match(f"    FRED = 17,  {marker} a comment", ("FRED", "17", "a comment"))
                self.assert_match(f"    FRED = 0x1A,  {marker} a comment", ("FRED", 26, "a comment"))
                self.assert_match(f"    FRED = (1U << 3),  {marker} a comment", ("FRED", 8, "a comment"))

    def test_comment_text_resembling_a_marker_is_kept(self):
        for text in ["!important", "<not a tag", "<< operator", "/usr/bin", "see https://ardupilot.org"]:
            with self.subTest(text=text):
                self.assert_match(f"    FRED,  //{text}", ("FRED", None, text))
                self.assert_match(f"    FRED = 0x1A,  //{text}", ("FRED", 26, text))

    def test_define(self):
        self.assert_match("#define FRED 3  // a comment", ("FRED", "3", "a comment"))

    def test_expressions_are_rejected(self):
        # partial matches would give a wrong value, so the whole line must match
        for line in [
            "    FRED = 0x18 + 1,",
            "    FRED = (1U<<4) | 8,",
            "    FRED = 0x1 << SHIFT,",
            "    FRED = 0x1, BARNEY = 0x2,",
            # unbalanced parentheses
            "    FRED = (1U << 3,",
            "    FRED = 1U << 3),",
            # comments in enumerations must use "//", not "/* */"
            "    FRED = 0x1A,  /* a comment */",
            "    FRED,  /* a comment */",
        ]:
            with self.assertRaises(ValueError):
                self.docco.match_enum_line(line)

    def test_unparsed_numbers_are_rejected(self):
        # a numeric value which is not understood must not be silently
        # discarded along with its whole enumeration
        for line in [
            "    FRED = 0b101,",
            "    FRED = 1e3,",
            "    FRED = 17Q,",
            "    FRED = 0x1AZ,",
            "    FRED = 01 << 2,",
            "    FRED = 1 << 010,",
            # octal
            "    FRED = 077,",
            "    FRED = 00,",
            # -1U is UINT_MAX, not -1
            "    FRED = -1U,",
            "    FRED = -1ul,",
            # the result depends on the width of the base's type
            "    FRED = 3U << 31,",
            "    FRED = (1U << 32),",
            "    FRED = 1 << 40,",
            "    FRED = 3ULL << 63,",
        ]:
            with self.subTest(line=line):
                with self.assertRaises(ValueError):
                    self.docco.match_enum_line(line)

    def test_non_literal_values_are_discarded(self):
        # these are understood but cannot be evaluated; the enumeration is skipped
        self.assert_match("    FRED = BARNEY,", (None, None, None))
        self.assert_match("    FRED = EVENT_MASK(1),", (None, None, None))
        self.assert_match("    FRED = FUNC(1, 2),", (None, None, None))


class TestEnumerationsFromFile(unittest.TestCase):
    """Tests for extracting whole enumerations from a source file."""

    def enumerations(self, content):
        (fd, path) = tempfile.mkstemp(suffix=".h")
        try:
            with os.fdopen(fd, "w") as f:
                f.write(content)
            return {e.name: e.entries for e in EnumDocco(None).enumerations_from_file(path)}
        finally:
            os.unlink(path)

    def test_class_qualified_name_and_values(self):
        enums = self.enumerations('''
class Fred {
    enum DevTypes {
        DEVTYPE_A = 0x01,
        DEVTYPE_B = 0x02,
    };
};
''')
        self.assertIn("Fred::DevTypes", enums)
        entries = enums["Fred::DevTypes"]
        self.assertEqual([(e.name, e.value) for e in entries], [("DEVTYPE_A", 1), ("DEVTYPE_B", 2)])

    def test_enum_class_with_underlying_type(self):
        enums = self.enumerations('''
class Fred {
    enum class DevType : uint8_t {
        A = 0x01,
        B,
    };
};
''')
        entries = enums["Fred::DevType"]
        self.assertEqual([(e.name, e.value) for e in entries], [("A", 1), ("B", 2)])

    def test_comment_containing_brace_does_not_end_enumeration(self):
        enums = self.enumerations('''
class Fred {
    enum DevTypes {
        DEVTYPE_A = 0x01,
        DEVTYPE_B = 0x02,  // do not write "};" here
        DEVTYPE_C = 0x03,
    };
};
''')
        entries = enums["Fred::DevTypes"]
        self.assertEqual([e.name for e in entries], ["DEVTYPE_A", "DEVTYPE_B", "DEVTYPE_C"])
        self.assertEqual(entries[1].comment, 'do not write "};" here')

    def test_commented_out_entry_is_ignored(self):
        enums = self.enumerations('''
class Fred {
    enum DevTypes {
        DEVTYPE_A = 0x01,
        // DEVTYPE_B = 0x02,  // retired, do not reuse
    };
};
''')
        self.assertEqual([e.name for e in enums["Fred::DevTypes"]], ["DEVTYPE_A"])

    def test_enumeration_with_unevaluatable_entry_is_skipped(self):
        enums = self.enumerations('''
class Fred {
    enum DevTypes {
        DEVTYPE_A = 0x01,
        DEVTYPE_ALIAS = DEVTYPE_A,
    };
};
''')
        self.assertNotIn("Fred::DevTypes", enums)

    def test_suffixed_value_does_not_drop_enumeration(self):
        enums = self.enumerations('''
class Fred {
    enum class Use : uint8_t {
        NONE = 0U,
        ALL = (1U<<0),
        HEX = 0x04L,
    };
};
''')
        self.assertEqual([(e.name, e.value) for e in enums["Fred::Use"]], [("NONE", 0), ("ALL", 1), ("HEX", 4)])

    def test_single_line_enumeration(self):
        enums = self.enumerations('''
class Fred {
    enum class DistanceMode { Short, Medium, Long, Unknown };
    enum Explicit { A = 5, B = 9, C };
    enum Skipped { D = BARNEY, E };
    enum SkippedCall { H = FUNC(1, 2), I };
    /* a comment */ enum AfterComment { F, G };
};
''')
        self.assertEqual([(e.name, e.value) for e in enums["Fred::DistanceMode"]],
                         [("Short", 0), ("Medium", 1), ("Long", 2), ("Unknown", 3)])
        self.assertEqual([(e.name, e.value) for e in enums["Fred::Explicit"]], [("A", 5), ("B", 9), ("C", 10)])
        self.assertNotIn("Fred::Skipped", enums)
        self.assertNotIn("Fred::SkippedCall", enums)
        self.assertEqual([(e.name, e.value) for e in enums["Fred::AfterComment"]], [("F", 0), ("G", 1)])

    def test_single_line_enumeration_error_names_file_and_line(self):
        with self.assertRaisesRegex(ValueError, r"\.h:3: Failed to match"):
            self.enumerations("class Fred {\n\n    enum X { A = 0x18 + 1, B };\n};\n")

    def test_code_after_block_comment_is_kept(self):
        # losing B would also give C the wrong implicit value
        for comment in ["/* a comment */", "/* a\n       multi-line comment */"]:
            with self.subTest(comment=comment):
                enums = self.enumerations(f'''
class Fred {{
    enum DevTypes {{
        A,
        {comment} B,
        C,
    }};
}};
''')
                self.assertEqual([(e.name, e.value) for e in enums["Fred::DevTypes"]], [("A", 0), ("B", 1), ("C", 2)])

    def test_block_comment_followed_by_line_comment(self):
        enums = self.enumerations('''
class Fred {
    enum DevTypes {
        A,
        /* a comment */  // another comment
        B,
    };
};
''')
        self.assertEqual([(e.name, e.value) for e in enums["Fred::DevTypes"]], [("A", 0), ("B", 1)])

    def test_unterminated_block_comment_raises(self):
        # this used to loop forever
        with self.assertRaisesRegex(ValueError, r":5: unterminated /\* comment"):
            self.enumerations("class Fred {\n    enum X { A,\n    };\n};\n/* no end\n")

    def test_parse_error_names_file_and_line(self):
        with self.assertRaisesRegex(ValueError, r"\.h:4: Failed to match"):
            self.enumerations('''
class Fred {
    enum DevTypes {
        A = 0x18 + 1,
    };
};
''')

    def test_block_comment_on_entry_gives_hint(self):
        with self.assertRaisesRegex(ValueError, r"\.h:4: .*use // rather than /\* \*/"):
            self.enumerations('''
class Fred {
    enum DevTypes {
        A = 0x01,  /* a comment */
    };
};
''')

    def test_no_block_comment_hint_for_line_comment(self):
        with self.assertRaises(ValueError) as cm:
            self.enumerations('''
class Fred {
    enum DevTypes {
        A = 0x18 + 1,  // not a /* block */ comment
    };
};
''')
        self.assertNotIn("use //", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
