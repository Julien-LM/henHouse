Existant :

- Communication à une RTC, HW084 (module pour DS3231) (pluggué sur la carte via un connecteur femelle pour CI 4 type SSW-110-02-G-S)
- Commande du moteur à courant continu HW095 (Carte pour L298N) (3 sortie de bornier, pour raccorder la carte déportée)
- Capteurs de fin de course haut et bas (uniquement le capteur deporté, une sortie de bornier sur la carte + circuit de contrôle)
- Led d'erreur et led d'avertissement commandé par des transistor 2N2222 (Led + transistors déportés, uniquement une sortie de bornier sur la carte)
- un bouton pour la commande manuel d'ouverture/fermeture de la porte
- Utilisation de borniers CI de type 282834-5 (TE-connectivity)

- Prévoir également un bornier pour la masse et le +5V

Prévisions :

- Fuel gauge baterie 12V
- Commande arrosage serre
- Commande de la cloture
- Commande fontaine bassin

Futur :

- Communication avec un esp32
- Mesure niveau cuve eau (capteur trop plein)
- Afficheur LCD (I²C?)
- I/O expandeur (SPI/I²C?)
- Boutons commande (3/4)
- Capteur température intérieur extérieur
