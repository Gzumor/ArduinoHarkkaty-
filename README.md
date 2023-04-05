# ArduinoHarkkatyö
DC moottorin ja servon ohjaus joystickillä ja sykemittarilla

Harjoitustyö on osa Elektroniikka ja Ohjelmointi kurssia koneautomaation insinööriopinnoissa.
Tehtävänä oli kirjoittaa ohjaus Arduinolle servon ajamiseen joystickillä ja automaattisesti laidasta laitaan erillistä automaattitilan nappia pianettaessa.
Servon toimintaa parannettu vaaditulla ryntäyksenestolla, mutta myös lisätty ylimääräisenä automaattiajolta manuaaliajolle palautumisen pehmennys joystickin 
asettamaan tavoitekulmaan.

Toinen vaadittu ominaisuus oli ajastimen rakentaminen. Ajastin tallentaa ohjelman "standby" tilassa kulunutta aikaa ja tulostaa sitä sekunnin välein. Lisäominaisuutena ajan tallennus EEPROMilla Arduinon muistiin, jolloin aika säilyy vaikka laite resetoidaan tai virrat katkaistaan.

Lisätehtävänä oli koodata ohjaus DC moottorille. DC moottorin nopeus on säädettävissä joystickin Y-akselilla manuaalitilassa. Moottorinohjaukseen lisätty toiminnallisuutta
vaaditun lisäksi suunnanvaihdolla, joka kääntyy joystickin keskiasennon poikkeaman suunnalla, jolloin rele aktivoituu ja moottorin napaisuus kääntyy. 
 Moottorin ohjaukseen lisätty automaattiajolla nopeuden säätö sykeanturin avulla. Moottorin nopeus inkrementoidaan joystickin tai sykkeen asettamaan MOSFETille skaalattuun tavoitearvoon.
 
Koodissa ei käytetä runkoloopin lisäksi yhtään looppirakennetta tai delaytä, jolloin usean järjestelmän ajaminen Arduinolla yhtäaikaisesti on mahdollista.
