# Closest-Building-OSM
C++ program that finds the closest meetup destination between two different buildings. \
\
Building data is extracted from .osm file \
\
Implements Dijkstra's Algorithm to find the closest possible building destination between the user's two choices.

## Demonstration
In the project directory run:
```bash
./application.exe
```
You will be prompted with 
```bash
"Enter map filename>"
```
Input the name of a single .osm file (uic.osm & uiuc.osm are provided with the project)
```bash
"Enter map filename>" uic.osm
```

Choose either the standard application ('a') or creative component('c') \
\
Input one building at a time \
![image](https://user-images.githubusercontent.com/88940506/186742242-afb2f819-1fa4-4912-8358-bcced8110627.png)

If there are no mistakes in your input, you will receive the following formatted output
![image](https://user-images.githubusercontent.com/88940506/186742355-6d7616c4-8ceb-4b1b-9d2b-18539a18782f.png)
