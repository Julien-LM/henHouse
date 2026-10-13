# HenHouse
Automatic door

Note:

V1.0 => Porte fonctionnelle
Commande d'ouverture via la RTC
Affichage via LED en cas d'erreurs



Next : Réalisation d'un PCB propre avec des ajouts :
Communication SPI/I2c avec le ESP 32
Ajout d'une fuel gauge
Libération d'entrée sorties ou ajout d'un mltiplexeur pour permettre la commande de relais pour (eclairage/ Arrosage/ ouverture/fermeture/clutore)

Nouveau choix technique, passage de l'arduino à un microcontrôleur microchip PIC18F47Q43
Ajoût d'un régulateur à faible consommation MCP1702 en TO-92 pour stabiliser le 5V. On conserve le LM2596 pour descendre le 12V à 6V efficacement 
