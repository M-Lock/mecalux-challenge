# Mecalux-challenge

## WAREHOUSE OPTIMIZATION ESP

El algoritmo diseñado está basado en un problema de similar naturaleza denominado Bin packing problem, conocido por ser un algoritmo de tipo NP-hard, es decir, computacionalmente imposible de dar solución óptima en un tiempo razonable. Por ello, hemos decidido desarrollar una Metaheurística GRASP (Greedy Randomized Adaptive Search Procedure) y aplicarla a un grid. Para ello, realizamos los siguientes pasos:

1. **Preparación de la cuadricula**: Rellenamos el grid con zonas ilegales (1) y zonas permitidas (0), diferenciando así obstaculos, bordes del almacén y limite en el eje Z con el techo. Tenemos una variable `res` la cuál es clave para la eficiencia del algoritmo, ya que permite calcular el número de cuadros que tendrá el grid dinámicamente, en función del tamaño del almacén.
2. **Heurística Greedy**: Aplicando una heurística relación precio/carga, ordenamos las bahías de forma que seleccionaremos las más rentables primero. Tras esto, entramos al bucle principal:

3. **Ejecución del bucle**: Dentro del bucle, que se ejecuta durante 29 segundos, tenemos la decisión crucial cada iteración de a.) trabajar en la solución optima durante una ejecución para mejorarla b.) probar con una nueva desde el principio, para ver si conseguimos llegar a una solución más eficiente. Una vez tomada la decisión, si estamos en un nuevo intento, tenemos una función que coloca las bahías, calculando en cada rotación para probar si esto mejora la ejecución. Además, si consigue colocar la bahía, marca los gaps que esta deja de forma que las siguientes bahías, si pueden ocupar dicho espacio superponiendose, reciben una puntuación extra. Cuando se llena (o se selecciona mejorar la solución existente) se realizan un conjunto de operaciones para tratar de mejorarla, pegando las paredes a bordes buscando disminuir espacios vacíos, tratando de meter alguna otra bahía con la función anterior, ó eliminando cierta cantidad de bahías para probar nuevas permutaciones.

4.**Evaluación de Costes**: A cada cambio de las funciones anteriores dentro del bucle, calculamos el resultado de la función matemática propuesta que debemos **minimizar**. Esto nos permite mantener siempre la mejor versión del almacén a la que hayamos llegado durante la ejecución.


## WAREHOUSE OPTIMIZATION ENG

The designed algorithm is based on a problem of a similar nature called the Bin packing problem, known for being an NP-hard problem, meaning it is computationally impossible to provide an optimal solution in a reasonable amount of time. Therefore, we decided to develop a GRASP (Greedy Randomized Adaptive Search Procedure) metaheuristic and apply it to a grid. To do this, we perform the following steps:

1. **Grid preparation**: We fill the grid with illegal zones (1) and permitted zones (0), thus differentiating obstacles, warehouse borders, and the Z-axis limit with the ceiling. We have a `res` variable which is key to the efficiency of the algorithm, as it allows to calculate the amount of squares on the grid dinamicly, depending on the warehouse's size.
2. **Greedy Heuristic**: Applying a price/load ratio heuristic, we sort the bays so that we select the most profitable ones first. After this, we enter the main loop:

3. **Loop execution**: Inside the loop, which runs for 29 seconds, we have the crucial decision in each iteration to either a.) work on the optimal solution during an execution to improve it, or b.) try a new one from scratch, to see if we can reach a more efficient solution. Once the decision is made, if we are in a new attempt, we have a function that places the bays, calculating every rotation to test if this improves the execution. Furthermore, if it manages to place the bay, it marks the gaps it leaves so that subsequent bays, if they can occupy that space by overlapping, receive an extra score. When it is filled (or if improving the existing solution is selected), a set of operations is performed to try to improve it, pushing the walls against the borders seeking to reduce empty spaces, trying to insert another bay with the previous function, or eliminating a certain number of bays to test new permutations.

4. **Cost Evaluation**: At every change made by the previous functions within the loop, we calculate the result of the proposed mathematical function that we must **minimize**. This allows us to always keep the best version of the warehouse that we have reached during the execution.
