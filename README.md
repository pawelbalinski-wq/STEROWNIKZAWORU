# STEROWNIKZAWORU
Prosty sterownik pogodowy zaworu 3D mieszacza ogrzewania podłogowego na Arduino Uno.
Liniowa krzywa grzewcza dopasowana do mojej istniejącej instalacji. Zamierzeniem jest skonstruowanie sterownika jak najprostrzego w obsłudze dla nietechnicznej osoby. Aktualnie używam sterownika TECH, sam się gubię w jego obsłudze i bardzo łatwo go "rozregulować".
Sterownik ma obsługiwać zawór mieszacza na podstawie krzywej grzewczej, dwie pompy obiegowe i funkcję ogrzewania kotła gazowego.
Obsługa sterownika klawiszami dedykowanymi do każdej funkcji, wyświetlacz dwuwierszowy z danymi: ustawiona temperatura dla krańców krzywej grzewczej, temperatury na wejściu i wyjściu mieszacza. Dodatkowo diody sygnalizujące stan pomp i pozostałych funkcji zamiast symboli na wyświetlaczu.

Klawisz 1: krzywa grzewcza -15 stopni - zwiększenie o 0,5 stopnia
Klawisz 2: krzywa grzewcza -15 stopni - zmniejszenie o 0,5 stopnia
Klawisz 3: krzywa grzewcza +10 stopni - zwiększenie o 0,5 stopnia
Klawisz 4: krzywa grzewcza +10 stopni - zmniejszenie o 0,5 stopnia
Klawisz 5: tryb LATO + dioda sygnalizacyjna żółta
Klawisz 6: tryb ZIMA + dioda sygnalizacyjna biała
Klawisz 7: tryb wiosna/jesień + dioda sygnalizacyjna zielona
Klawisz 8: STOP + dioda sygnalizacyjna czerwona

Wyświetlacz górny rząd: po lewej zadana temperatura dla -15 stopni, po prawej zadana temperatura dla +10 stopni
Wyświetlacz dolny rząd: po lewej temperatura na wejściu mieszacza, po prawej temperatura na wyjściu mieszacza

Program pracy:
tryb 
