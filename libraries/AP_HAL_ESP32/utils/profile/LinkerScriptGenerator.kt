import java.io.File
import java.io.FileOutputStream
import java.nio.charset.Charset
import java.util.*
import java.util.regex.Pattern
import kotlin.collections.ArrayList

data class Func(
    var symbol: String,
    var place: String,
    var address: Int,
    var size: Int,
    var count: Int
)

val p = Pattern.compile(" (?:\\.iram1|\\.text)+\\.(\\S*)\\s*(0x.{16})\\s*(0x\\S*)\\s*(\\S*)")
val placep = Pattern.compile("[^(]*\\(([^.]*)")

fun generateLinkerScript(mapFileName: String, profileFileName: String, scriptFileName: String) {
    var addressToFunction = TreeMap<Int, Func>()
    fun parseMap() {
        val s = File(mapFileName).readText(Charset.defaultCharset())
        val m = p.matcher(s)
        while (m.find()) {
            val symbol = m.group(1)
            val address = Integer.decode(m.group(2))
            val size = Integer.decode(m.group(3))
            var place = m.group(4)
            if (address == 0) {
                continue
            }
            var placem = placep.matcher(place)
            if (placem.find()) {
                place = placem.group(1)
            }
            var f = Func(symbol, place, address, size, 0)
            addressToFunction[f.address] = f
        }
    }

    fun parseProfile() {
        Scanner(File(profileFileName)).use { scanner ->
            while (scanner.hasNext()) {
                val address = Integer.decode(scanner.next())
                val count = Integer.decode(scanner.next())
                for(f in addressToFunction.values) {
                    if(f.address <= address && address < f.address + f.size) {
                        f.count = count
                    }
                }
            }
        }
    }

    fun writeScript() {
        var excl = listOf( "_ZN12NavEKF2_core15readRangeFinderEv",
        "_ZN12NavEKF2_core18SelectRngBcnFusionEv","_ZN12NavEKF2_core14readRngBcnDataEv")

        var lst = ArrayList(addressToFunction.values.filter { it.count > 5000 && !excl.contains(it.symbol) })
        lst.sortWith(kotlin.Comparator { o1, o2 -> o2.count - o1.count })

        var s = 0
        for (a in lst) {
            System.out.format(
                "0x%016x\t%8d\t%8d\t%s:%s\n",
                a.address,
                a.size,
                a.count,
                a.place,
                a.symbol
            )
            s += a.size
        }
        FileOutputStream(scriptFileName).use {
            for (a in lst) {
                it.write(String.format("%s:%s\n", a.place, a.symbol).toByteArray())
            }
        }
        System.out.format("total: %d\n", s)
    }
    parseMap()
    parseProfile()
    writeScript()
}

fun main(args: Array<String>) {
    generateLinkerScript(
        "arduplane.map",
        "PROF000.TXT",
        "functions.list"
    )
}