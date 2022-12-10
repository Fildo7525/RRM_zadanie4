# Zadanie4 z predmetu Riadenie robotickych manipulatorov

## Upesnenie

Vsetky grafy v [ulohe jedna](#Uloha1) su merane v case 0 az 4 sekundy. Pricom poloha klbov je v radianoch, ich rychlost je v rad/s, zrychlenie v rad/s^2
 a trh v rad/s^3. Podobne aj v [ulohe dva](#Uloha2) su vsetky grafy merane v case 0 az 9 sekund. V tychto grafoch zanzornujeme polohu 'x'-ovej a 'y'-ovej suradnice v kartezianskom systeme robota.

## Uloha 1

Zadajte do terminalu ```git switch zadanie1```. Po prehodeni sa druhej branche skompilujte program pomocou ```catkin_make```. Ubezpecte sa, ze mate stiahnuty Qt5 na vykreslovanie grafou. (```sudo apt install qt5-default libqt5charts5-dev```). Po skompilovani spustite program. Predpokladame, ze workspace, v ktorom sa dany program spusta je vytvoreny a source-nuty. Retazec na spustenie programu ```roslaunch abb_moveit_config demo.launch``` v jednom terminali a v druhom ternimali spustime ```rosrun trajectory_visualization trajectory_visualization```. Program nam najprv ukaze grafy pohybu prveho a tretieho klbu. Ked ich zavrieme spusti sa simulacia.

## Uloha 2

Zadajte do terminalu ```git switch zadanie2```. Po prehodeni sa druhej branche skompilujte program pomocou ```catkin_make```. Ubezpecte sa, ze mate stiahnuty Qt5 na vykreslovanie grafou. (```sudo apt install qt5-default libqt5charts5-dev```). Po skompilovani spustite program. Predpokladame, ze workspace, v ktorom sa dany program spusta je vytvoreny a source-nuty. Retazec na spustenie programu ```roslaunch abb_moveit_config demo.launch``` v jednom terminali a v druhom ternimali spustime ```rosrun trajectory_visualization trajectory_visualization```. Program nam najprv ukaze grafy polohy, rychlosti a zrychlenia pre 'z' suradnice toolu a grafy polohy, rychlosti a zrychlenia pre 'y' suradnice toolu. V druhom grafe vidime aj pohyb rotacie toolu okolo os 'z' Ked ich zavrieme spusti sa simulacia.

