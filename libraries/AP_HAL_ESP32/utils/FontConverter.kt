import java.io.File
import java.io.Writer
import java.util.*
import kotlin.collections.ArrayList

const val char_width = 12
const val char_height = 18


data class Symbol(
    var code: Int,
    var levl: ArrayList<Boolean>,
    var mask: ArrayList<Boolean>
)

var symbols = ArrayList<Symbol>()

fun readSymbols(fileName: String) {
    var data = ArrayList<Int>()
    Scanner(File(fileName)).use { scanner ->
        scanner.next()
        while (scanner.hasNext()) {
            data.add(Integer.parseInt(scanner.next(), 2))
        }
    }
    for (ch in 0 until 256) {
        var levl = ArrayList<Boolean>()
        var mask = ArrayList<Boolean>()
        for (y in 0 until char_height) {
            for (x in 0 until char_width) {
                var bitoffset = (y * char_width + x) * 2
                var byteoffset = 64 * ch + bitoffset / 8
                var bitshift = 6 - (bitoffset % 8)
                var v = (data[byteoffset] ushr bitshift) and 3
                levl.add(v and 2 != 0)
                mask.add(v and 1 == 0)

            }
        }
        symbols.add(Symbol(ch, levl, mask))
    }
}

fun printSymbols(fileName: String) {
    File(fileName).bufferedWriter().use { out ->
        out.write("#include <AP_OSD/AP_OSD_INT.h>\n")
        out.write("#ifdef WITH_INT_OSD\n")

        out.write("const uint32_t AP_OSD_INT::font_data[256 * 18] = {\n")
        for (code in 0 until 256) {
            val s = symbols[code]

            for (y in 0 until char_height) {
                var v: Int = 0
                var gr = ""
                for(x in 0 until char_width) {
                    if(s.levl[y * char_width + x]) {
                        v = v or (1 shl (31 - x))
                        gr += "*"
                    } else {
                        gr += "-"
                    }
                }
                out.write("0x%08x,\t//\t%s\n".format(v, gr))
                print(gr + "\n")
            }
            out.write("\n")
            print("\n")
        }
        out.write("};\n#endif\n")
    }
}

fun main() {
    readSymbols("bold.mcm")
    printSymbols("Symbols.cpp")
}

