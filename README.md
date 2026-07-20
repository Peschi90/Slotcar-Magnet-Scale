# Slotcar Magnet Scale mit 2x HX711 (Wemos D1 mini)

Dieses Projekt liest zwei Waegezellen aus, die an den gegenueberliegenden Enden einer gemeinsamen langen Platte montiert sind.
Die Lastverteilung zwischen links/rechts aendert sich je nach Position der Kraft, das Gesamtgewicht wird aber aus einer gewichteten Kombination beider Sensoren berechnet.

Formel der Gesamtmessung:

gewichtGramm = faktorLeft * netLeft + faktorRight * netRight

Dabei gilt:

- netLeft = rawLeft - offsetLeft
- netRight = rawRight - offsetRight

Die Faktoren werden nicht einzeln pro Zelle kalibriert, sondern gemeinsam ueber mehrere Positionen mit demselben Referenzgewicht.

## Verdrahtung

### Linker HX711

- DOUT / DT -> D1 / GPIO5
- SCK / CLK -> D2 / GPIO4
- VCC -> 3V3
- GND -> GND

### Rechter HX711

- DOUT / DT -> D5 / GPIO14
- SCK / CLK -> D6 / GPIO12
- VCC -> 3V3
- GND -> GND

## Mechanischer Aufbau

Beide Waegezellen sind fest mit derselben Platte verbunden. Dadurch ist eine echte Einzelkalibrierung pro Zelle mechanisch nicht moeglich.
Wenn die Kraft links angreift, steigt der linke Anteil und der rechte Anteil sinkt (und umgekehrt).
Nur die gemeinsame Mehrpunktkalibrierung kann diese Lastverteilung korrekt in stabile Grammwerte umrechnen.

## Warum mehrere Positionen zwingend sind

Wenn das Referenzgewicht nur an einer Position gemessen wird, gibt es unendlich viele Faktorpaare, die dort passen.
Erst durch mehrere deutlich unterschiedliche Positionen (z. B. weit links, Mitte, weit rechts) wird das Gleichungssystem ausreichend bestimmt.
Die Firmware loest die Faktoren mit Least Squares aus allen aufgenommenen Punkten.

## PlatformIO

1. Projekt in VS Code mit PlatformIO oeffnen.
2. Wemos D1 mini per USB verbinden.
3. Firmware hochladen (Upload).
4. Seriellen Monitor mit 115200 Baud starten.
5. Zeilenende CR, LF oder CRLF verwenden (alles wird unterstuetzt).

## Schritt-fuer-Schritt-Kalibrierung

1. Platte komplett entlasten.
2. Tara setzen:

	TARA 20

3. Kalibriermodus mit bekanntem Referenzgewicht starten, Beispiel 1000 g:

	KAL_START 1000

4. Dasselbe Gewicht nacheinander an mehreren Positionen platzieren und jeweils Punkt aufnehmen:

	KAL_PUNKT LINKS 20
	KAL_PUNKT MITTE 20
	KAL_PUNKT RECHTS 20

5. Punkte optional pruefen:

	KAL_LISTE

6. Faktoren berechnen und aktivieren:

	KAL_BERECHNEN

Nach erfolgreichem KAL_BERECHNEN werden die Faktoren automatisch im EEPROM gespeichert.

## Beispielkalibrierung mit 1000 g

TARA 20
KAL_START 1000
KAL_PUNKT LINKS 20
KAL_PUNKT MITTE 20
KAL_PUNKT RECHTS 20
KAL_BERECHNEN

Anschliessend pruefen:

MESSEN 10

## Befehlsuebersicht

- HILFE
- STATUS
- WLAN <ssid> <passwort>
- WLAN_LOESCHEN
- TARA [messungen]
- KAL_START <gewicht_g>
- KAL_PUNKT <name> [messungen]
- KAL_LISTE
- KAL_LOESCHEN
- KAL_BERECHNEN
- KAL_ABBRUCH
- MESSEN [messungen]
- ROH [messungen]
- START [intervall_ms] [messungen]
- STOP
- INTERVALL <ms>
- FAKTOREN <faktor_links> <faktor_rechts>
- SPEICHERN
- RESET

## WLAN Verhalten

- Beim Start wird, falls konfiguriert, eine STA-Verbindung zum gespeicherten WLAN versucht.
- Falls keine Verbindung zustande kommt, startet automatisch der Backup-AP:
	- SSID: Slotcar-Magnet-Scale
	- Passwort: slotcar123
- STATUS zeigt den aktuellen WLAN-Modus (STA, AP oder AP+STA) und ob eine STA-Verbindung aktiv ist.

## Weboberflaeche

- Die Firmware stellt wieder eine lokale Weboberflaeche bereit.
- Aufruf im Browser:
	- im Heimnetz ueber die STA-IP (siehe STATUS)
	- oder im Backup-AP ueber http://192.168.4.1
- Die Web-App hat wieder eine ausklappbare Menueleiste mit Seitenbereichen:
	- Dashboard
	- Messung
	- Kalibrierung
	- WLAN
	- System
- Die alte Kalibrierungs-Route /kalibrierung bleibt erhalten und springt direkt auf den Kalibrierungsbereich.
- WLAN-Seite enthaelt wieder:
	- WLAN-Netzwerke suchen (Scan)
	- SSID aus gefundener Liste uebernehmen
	- WLAN speichern/loeschen
	- Verbindungsdetails (Modus, STA verbunden, STA-IP, AP-SSID, AP-IP)
- Kalibrierungsbereich zeigt die Punktliste live als Tabelle mit Soll/Berechnet/Abweichung.

## Ausgabeformate

MESSEN liefert mindestens:

- LINKS_ROH
- RECHTS_ROH
- LINKS_NETTO
- RECHTS_NETTO
- LINKS_ANTEIL / RECHTS_ANTEIL (wenn sinnvoll berechenbar)
- GEWICHT

Streammodus (START 500 5) liefert kompakt:

GEWICHT: 1000.42 g | LINKS: 61.8 % | RECHTS: 38.2 %

## EEPROM-Inhalt

Gespeichert werden:

- Offset links
- Offset rechts
- Faktor links
- Faktor rechts
- Tara vorhanden
- gueltige Kalibrierung vorhanden
- Konfigurationsversion
- Magic Number
- Pruefsumme

Temporare Kalibrierpunkte werden nicht gespeichert.

## Hinweise zur mechanischen Verspannung

- Die Platte sollte nicht verkanten oder seitlich klemmen.
- Befestigungen links/rechts moeglichst gleichartig ausfuehren.
- Kabel duerfen die Platte nicht mechanisch mitziehen.
- Bei starker Verspannung steigen die Kalibrierfehler deutlich.
- Wenn KAL_BERECHNEN grosse Restfehler meldet, zuerst Mechanik pruefen, dann Kalibrierung wiederholen.
