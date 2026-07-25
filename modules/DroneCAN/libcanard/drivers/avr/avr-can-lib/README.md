Universelle CAN Blibiothek (avr-can-lib)
========================================

Universelle CAN Bibliothek für AVRs. Unterstütz werden AT90CAN, MCP2515 und SJA1000.
Siehe dazu auch den Artikel [Universelle CAN Bibliothek][can_lib] auf [kreatives-chaos.com][].

Motivation
----------

Die Bibliothek ist für den [Roboterclub Aachen e.V.][rca] entstanden. Vor zwei 
Jahren stand ein Wechsel des internen Bussystems an, die Wahl fiel dabei auf den
CAN-Bus, da er eigentlich ideal für diese Anwendung ist.  
Es gab nur leider keine Open-Source Bibliotheken für die Ansteuerung der 
gebräuchlichsten CAN-Controller, so dass selbst schreiben angesagt war.

Die Bibliothek ist jetzt seit einiger Zeit in Gebrauch und wurde dabei vom 
[MCP2515][] auf die anderen CAN-Controller erweitert.


Features
--------

* Unterstützung des MCP2515
* Unterstützung des SJA1000 (für AVRs mit oder ohne externem Bus-Interface!)
* Unterstützung der AT90CAN-Reihe
* Aufbau als Library, damit werden nur die benötigen Funktionen verwendet
* geringer Resourcen-Verbrauch

Die Bibliothek braucht ca. 1500 Byte Flash mit allen Funktionen. Verwendet man 
keine dynamischen Filter sind es 370 Byte weniger.

### Filter

Für den MCP2515 gibt es zwei Arten die Masken und Filter anzusprechen, zum 
einen "statisch" mit einer Liste die zur Compile-Zeit angelegt wird oder 
"dynamisch" während der Laufzeit, dann aber mit größerem Flash Verbrauch.

Die AT90CANxxx kennen nur die dynamischen Filter.

Gleiches gilt für den SJA1000. Dieser bietet zwar theoretisch zwei Filter für 
Standard-Nachrichten bzw. einen für Extended-Nachrichten, in der Praxis sind 
diese aber mehr oder weniger unbrauchbar.


Einbinden ins eigene Programm
-----------------------------

Zuerst einmal muss die Bibliothek erstellt werden: Dazu ändert man im 
`src/`-Ordner die Datei 'config.h' für seine Konfiguration ab und stellt im 
Makefile den verwenden AVR-Typ ein.

Dann kann die Bibliothek gebaut werden:

    $ make lib

Wer WinAVR verwendet kann auch einfach im Programmers Notepad eine der Dateien 
aus dem `src/`-Ordner öffnen und dann "Tools > [WinAVR] Make All" auswählen.

Jetzt sollte eine Datei 'libcan.a' entstanden sein. Zusammen mit 'can.h' und 
'config.h' können diese Dateien jetzt in das eigene Projektverzeichnis kopiert
werden.

Um die Funktionen wirklich nutzten zu können muss nur noch den Linker mitgeteilt
werden, dass er die Bibliothek einbinden soll. Bei dem Standard-WinAVR-Makefile
muss dazu die folgende Zeile hinzugefügt werden (um Zeile 266ff):

    ...
    LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
    LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
    
    LDFLAGS += -L. -lcan        # hinzufügen

Der Linker sucht dann automatisch im aktuellen Verzeichnis nach der Datei 
libcan.a und fügt die benötigten Funktionen (und nur die) in den Quellcode ein.


#### Hinweise zur Benutzung der Ports:

Da bei AVR's die I/O Pins mehrfach belegt sind, beachten Sie bitte unbedingt die
Fusebits des von Ihnen eingesetzten AVR's, konkret am Beispiel ATmega32:

Port C teilt sich Bit 2 - 5 mit der JTAG Schnittstelle, die ab Werk aktiviert ist - deaktivieren!  
Port D teilt sich Bit 0 und 1 mit der seriellen Schnittstelle. Wenn Sie alle Pins verwenden wollen, dürfen Sie den MAX 232 nicht bestücken.

Wenn man dann noch die Quelle der Oszillatorfrequenz richtig setzt, kann man fröhlich CANnen:

ATmega32  Low Fuse: 0xEF (External Crystal)
ATmega32 High Fuse: 0xD9 (JTAG Disabled) 


Testprogram
-----------

Ein Program was einfach eine Nachricht per CAN verschickt könnte folgendermaßen
aussehen:

	int main(void)
	{
		// initialisieren des MCP2515
		can_init(BITRATE_125_KBPS);
	
		// erzeuge eine Testnachricht
		can_t msg;
	
		msg.id = 0x123456;
		msg.flags.rtr = 0;
		msg.flags.extended = 1;
		
		msg.length = 4;
		msg.data[0] = 0xde;
		msg.data[1] = 0xad;
		msg.data[2] = 0xbe;
		msg.data[3] = 0xef;
		
		// Nachricht verschicken
		can_send_message(&msg);
	
		while (1) {
   	     
		}
	}


Lizenz
------

Die Bibliothek steht unter einer [BSD-Lizenz][] und darf damit frei verwendet
werden.

Wenn jemand Änderungen vornimmt würde ich bitten mir oder jemand anderem aus
dem [Roboterclub][rca] diese zukommen zu lassen, damit wir sie gegebenenfalls
in die Software einbauen können, so dass dann alle etwas davon haben.

[can_lib]: http://www.kreatives-chaos.com/artikel/universelle-can-bibliothek
[kreatives-chaos.com]: http://www.kreatives-chaos.com
[rca]: http://www.roboterclub.rwth-aachen.de/
[MCP2515]: http://www.kreatives-chaos.com/artikel/ansteuerung-eines-mcp2515
[BSD-Lizenz]: http://de.wikipedia.org/wiki/BSD-Lizenz