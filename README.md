# Engineering-Thesis
Comparative analysis of mobile robot simulators in Python.

[PL]
Niniejszy projekt inżynierski koncentruje się na analizie i ocenie popularnych 
symulatorów robotycznych pod kątem ich możliwości, funkcjonalności oraz pracy z 
językiem Python. W ramach projektu przeanalizowano cztery symulatory: Webots, 
CoppeliaSim, PyBullet oraz Gazebo. Dla każdego z nich zrealizowano autorskie 
symulacje, obejmujące różne scenariusze. Na podstawie przeprowadzonych testów 
dokonano szczegółowego porównania narzędzi, uwzględniając takie aspekty, jak wygoda 
pracy, intuicyjność interfejsu, biblioteki modeli oraz integracja z Pythonem. Ostateczne 
wnioski pozwalają na wybór symulatora odpowiedniego do specyficznych wymagań i 
celów projektu.

[EN]
This engineering project focuses on the analysis and evaluation of popular robotic 
simulators in terms of their capabilities, functionalities, and compatibility with the Python 
programming language. The project examines four simulators: Webots, CoppeliaSim, 
PyBullet, and Gazebo. Custom simulations were developed for each simulator, covering 
various scenarios. Based on the conducted tests, a detailed comparison of the tools was 
performed, considering aspects such as ease of use, interface intuitiveness, model libraries, 
and Python integration. The final conclusions allow for the selection of a simulator 
suitable for specific project requirements and objectives.




1. Webots – zawiera 3 foldery symulacyjne. Foldery Jazda do celu i Omijanie 
przeszkód zawierają pliki utworzonych scenerii oraz skrypty sterujące w języku 
Python. Folder test zawiera testowy skrypt z prostą symulacją poruszania się 
robota. 
2. CoppeliaSim – zawiera 2 pliki symulacyjne, które po uruchomieniu wczytują całą 
scenerię wraz ze skryptami sterującymi w języku Python. Plik line_follower odnosi 
się do symulacji śledzenia linii przez robota mobilnego, natomiast plik test to 
prosta symulacja testowa. 
3. PyBullet – zawiera 3 skrypty symulacyjne gotowe do uruchomienia w dowolnym 
środowisku programistycznym obsługującym język Python. Plik main_0 to 
symulacja trenowania robota z podejściem klasycznym, bez zaawansowanych 
algorytmów. Pliki main_1 oraz main_2 to symulacje z wykorzystaniem algorytmu 
PPO.
4. Gazebo – zawiera folder ros2_ws, który jest pełnym folderem z wszystkimi plikami 
niezbędnymi do zbudowania pakietu i uruchomienia symulacji w środowisku 
Gazebo. Dodatkowo załączono sam skrypt sterujący robot_controller w języku 
Python, który używany jest w symulacji omijania przeszkód. 
