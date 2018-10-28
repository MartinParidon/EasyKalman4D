Der C code ist im Wesentlichen eine einfache Übersetzung des Matlab Codes folgender Webseite:
http://www.cbcity.de/das-kalman-filter-einfach-erklaert-teil-2

Im master branch ist der funktionierende Code. Die anderen kannst du ignorieren.

Zwei Algorithmen wurden von woanders kopiert, ist dort auch gekennzeichnet. Könnte man sicher auch als Bibliothek bauen und einbinden, aber so ist es irgendwie offener.

Du brauchst mingw und gnuplot. Wenn sonst noch was fehlt sag bescheid.

CleanMakeAndRun macht genau das was der Name sagt (Vorausgesetzt die Abhängigkeiten passen). Neu bauen kann man direkt im gleichen Terminal durch Enter. Den Gnuplot kann man mit dem Matlab output vergleichen.