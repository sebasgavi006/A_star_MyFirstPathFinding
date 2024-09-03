/*
 ============================================================================
 Name        : A_star_pathfinding.c
 Author      : namontoy
 Version     :
 Copyright   : Programming to learn A*
 Description : Learning how to write my first A*...
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Single_Cell.h"

#define MAP_GRID_ROWS		8
#define MAP_GRID_COLS		10
#define MAP_CELLS_COUNT		(MAP_GRID_ROWS * MAP_GRID_COLS)
#define ROW_MAP_DATA_LEN	20
#define MAX_NEIGHBOURS		8

/* Elementos del sistema */
uint8_t grid_rows = MAP_GRID_ROWS;
uint8_t grid_cols = MAP_GRID_COLS;

Cell_map_t grid_map_cells[MAP_CELLS_COUNT] = {0};
Cell_map_t empty_cell = {0};
Cell_map_t *ptr_current_cell;
Cell_map_t *ptr_goal_cell;
Cell_map_t *ptr_start_cell;

Cell_map_t* open_list[MAP_CELLS_COUNT];
uint8_t open_list_index = 0;

Cell_map_t* closed_list[MAP_CELLS_COUNT];
uint8_t closed_list_index = 0;

char* map_string[MAP_GRID_ROWS];

/* Prototipos de las funciones del main */
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void print_cells_info(Cell_map_t *cellArray);
void print_single_cell_info(Cell_map_t *singleCell);
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map);
Cell_map_t* get_cell_start(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows);
void addTo_open_list(Cell_map_t *working_cell);
uint8_t identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check);
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
uint16_t get_G_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell);
uint16_t get_F_cost(Cell_map_t *working_cell);
void update_F_cost(Cell_map_t *working_cell);
void order_open_list(uint8_t index_last);
void init_empty_openlist(Cell_map_t* empty_cell);
uint8_t get_count_item_open_list(void);
void removeFrom_open_list(Cell_map_t *working_cell);
Cell_map_t* get_next_item(void);
void print_path(Cell_map_t *working_cell);
void A_star_algorithm(void);


/**
 * Función que ejecuta paso a paso el algoritmo A*
 * */
int main(void) {

	/* prints !!!Hello World!!! */
	printf("!!!Hello World!!!\n");

	printf("A* pathfinding\n");

	/* 1. Crea todas las celdas vacias */
	init_empty_grid_map(grid_cols, grid_rows, grid_map_cells);

	/* Estoy creando un "objeto" tipo cell_amp, el cual estará siempre vacio, para pruebas */
	empty_cell = create_cell(11,11);

	/* Inicializo todo el arreglo de punteros del open_list apuntando a la empty_cell */
	init_empty_openlist(&empty_cell);


	/* 2. Llena el mapa con la descripcion para el ejercicio
	 * En el caso del MCU, el string se debe recibir por el puerto serial,
	 * al igual que la indicación de a qué fila del mapa corresponde
	 * */
	populate_grid(". . . . . G . . . . ", 0, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 1, grid_map_cells);
	populate_grid(". . # # # # # # . . ", 2, grid_map_cells);
	populate_grid(". # # . . . . # # . ", 3, grid_map_cells);
	populate_grid(". # . . . . . . # . ", 4, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 5, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 6, grid_map_cells);
	populate_grid(". . . . S . . . . . ", 7, grid_map_cells);

	// Imprime la informacion mas simple de todas las celdas del grid
	//print_cells_info(grid_cels);

	/* 3. Imprime en pantalla el mapa que se envió a por los comandos del USART,
	 * para verificar que en efecto el sistema tiene el mapa correcto, o que el mapa
	 * fue correctamente recibido
	 * */
	print_map(grid_cols, grid_rows, grid_map_cells);

	/* 4. Ejecución del algoritmo A*
	 * Al llamar esta funcion, que basicamente ejecuta el pseudocodigo, se debe
	 * obtener al final la solución para la ruta.
	 * */
	A_star_algorithm();

/* == Espacio para pruebas simples... == */
//	/* Incluye la celda "Start" en la lista open_list, utilizando para esto al puntero "current_cell" */
//	addTo_open_list(ptr_start_cell);
//
//	print_single_cell_info(ptr_current_cell);
//	identify_cell_neighbours(grid_map_cells, &grid_map_cells[66]);
//	update_H_cost(ptr_goal_cell, &grid_map_cells[57]);
//	update_G_cost(ptr_current_cell, &grid_map_cells[57]);
//	update_F_cost(&grid_map_cells[57]);
//
//	addTo_open_list(&grid_map_cells[57]);
//	removeFrom_open_list(ptr_current_cell);
/* == Final de las pruebas simples... == */

	printf("END\n");



	return 0;
}

/**
 * Esta funcion se encarga de devolver la distancia "directa" entre la celda X en la que se
 * está trabajando y la celda definida como "Goal". En este primer caso será una heuristica
 * del tipo Pitagoras h = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ).
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los calculos con las funciones de CMSIS que con las
 * de C standar ya que son mucho mas eficientes.
 * */
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell){

    // Escribir código...

    return aux_result;
}

/*
 * Actualiza el valor del H_cost de la celda que se pasa como parametro, con respecto
 * a la celda "Goal" que se pasa tambien como parametro
 * */
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell){
	// Escribir código...
}

/**
 * Esta funcion se encarga de devolver la distancia "directa" entre la celda X y su siguiente
 * vecino, lo cual es el costo de viaje entre ambos. En este primer caso será una heuristica
 * del tipo Pitagoras G_cost_cell = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ) + G_cost_vecino.
 *
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los calculos con las funciones de CMSIS que con las
 * de C standar ya que son mucho mas eficientes.
 * */
uint16_t get_G_cost(Cell_map_t *neighbour_cell, Cell_map_t *working_cell){

	// Escribir código...

	return aux_result;
}

/*
 * Actualiza el valor del G_cost de la celda que se pasa como parametro, con respecto
 * a la celda vecina que se pasa tambien como parametro
 *
 * Para el caso especial de la celda "star" el G_cost siempre es 0 (ya que desde allí se inicia).
 * Esta caracterisitica debe quedar incluida en esta funcion.
 * */
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell){

    // Escribir código...
}


/**
 * Esta funcion se encarga de retornar el valor de la funcion de costo completa de la celda que se
 * está analizando.
 *
 * F = G_cost + H_cost
 *
 * */
uint16_t get_F_cost(Cell_map_t *working_cell){

	return // Escribir código...
}

/*
 * Actualiza el valor F_cost de la celda que se pasa como parámetro
 * */
void update_F_cost(Cell_map_t *working_cell){

	// Escribir código...

}


/**
 * Esta funcion busca cual es la celda que esta designada como el inicio (start)
 * y se apunta a ella con el putero "current_cell" para comenzar a hacer el analisis,
 * además se le organizan los parametros G, H y F adecuadamente...
 *
 * Este es de los primeros paso del pseudocódigo
 *
 * Se imprime un error si la celda no es encontrada en el arreglo.
 *
 * */
Cell_map_t* get_cell_start(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows) {

    // Escribir código...
    // Esta funcion retorna un valor...
}

/**
 * Esta funcion busca cual es la celda que esta designada como el "objetivo" (Goal),
 * además se le organizan los parametros G, H y F adecuadamente...
 * */
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows) {
    // Escribir código...
    // Esta funcion retorna un valor...
}

/**
 * Esta funcion debe organizar los elementos en la lista en orden, y el criterio es
 * primero F_cost y luego H_cost, si ambos son igual, se deja quien estaba primero.
 *
 * Este sistema implica que se debe poder mover hacia arriba y hacia abajo los elementos
 * dependiendo de la posicion que se le deba dar.
 *
 * Esta funcion utiliza un index, el cual solo puede ser modificado desde esta funcion
 * ya que este index se encarga de "mover" arriba y abajo el indicador de cuantos
 * elementos se encuentran abiertos.
 *
 * Además, tambien se tiene un puntero auxiliar, para poder comparar entre elementos y
 * saber quien debe ir en que posicion.
 *
 * Recordad que el elemento "open_list" es un arreglo de punteros de tipo Cell_map_t,
 * o sea esta lista NO almacena de nuevo las celdas, solo almacena la referencia a ellas
 * en el orden adecuado... punteros, queridos punteros...
 * */
void addTo_open_list(Cell_map_t *working_cell){

    // Escribir código...

}

/*
 * Entrega el puntero al elemento mas arriba de la lista open_list
 * */
Cell_map_t* get_next_item(void){

    return // Escribir código...

}

/**
 * Esta funcion identifica cuantos elementos activos hay en la lista open_list */
uint8_t get_count_item_open_list(void) {

	// Escribir código...
    // Esta funcion retorna un valor...
}
/*
 * Remueve el elemento indicado por el puntero working_cell, que ademas deberia ser el elemento
 * mas arriba en la lista de open_list
 * */
void removeFrom_open_list(Cell_map_t *working_cell){

    // Escribir código...

}

/*
 * Agrega el elemento pasado como parametro (puntero) en la lista de elementos cerrados
 * */
void addTo_closed_list(Cell_map_t *working_cell){

    // Escribir código...

}

/**
 * Carga la lista Open con el valor conocido de una celda vacia, y que además está fuera de las
 * fronteras del mapa. Esto lo hago con el objetivo de saber hasta donde debo recorrer el
 * arreglo open_list, de forma que pueda saber donde ya no hay nuevos elementos.

 * Es como la idea del caracter "NULL" en los strings...
 * */
void init_empty_openlist(Cell_map_t* empty_cell){

    // Escribir código...

}

/**
 * Funcion encargada de ordenear el arreglo OpenList con respecto a los valores F y H
 * de cada elemento al que se apunta en la lista.
 *
 * Esta función es "llamada" desde la función "addTo_open_list(...)"
 * */
void order_open_list(uint8_t index_last){

    // Escribir código...

}

/**
 * Un paso fundamental en el Algoritmo es identificar a los vecinos alrededor de la celda
 * que se encuentra activa en ese momento.
 * En este caso, deseo utilizar la posicion x,y como guia para encontrar los vecinos alrededor
 * de la celda[IDx] selecionada. Para esto vale la pena tener claro que los vecinos alrededor
 * de la celda son:
 * ???????????
 *
 * Viendo la celda X en una posicion general en el mapa:
 * ???????????
 *
 * Se obtiene asi entonces los vecinos de la celda activa.
 * */
uint8_t identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check){

    // Escribir código...

}

/**
 * Esta funcion deberia utilizar la información entregada de alguna manera,
 * como por ejemplo un string con todos los datos o quizas en un JSON.
 *
 * En la primer idea se emplea el envio de datos linea a linea (string con los
 * caracteres de cada fila (row) indicando además cual es la liena que se desea
 * actualizar (0 a 7).
 * */
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map){

    // Escribir código...

}

/**
 * Esta funcion se encarga de crear un grid_map vacio, solo con los
 * elementos que corresponden creados por defecto (vacios, sin costos y
 * sin indicar de qué tipo son)
 */
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray){

    // Escribir código...

}

/**
 * Me imrpime la información de una celda X, para verificar la informacion que ella contiene
 * y así poder mirar si todo esta funcionando correctamente.
 *
 * */
void print_single_cell_info(Cell_map_t *singleCell) {

    // Escribir código...

}

/**
 * Imprime la informacion de todas las celdas (en general del mapa)
 * */
void print_cells_info(Cell_map_t *cellArray){

    // Escribir código...

}


/**
 * Esta funcion se encarga de imprimir el mapa con sus caracteristicas
 * . default, empty cell
 * # obstacle
 * G Goal
 * S start
 * o Open
 * c Closed
 * El mapa se escribe fila a fila, por lo cual una idea es hacer fija la fila
 * e imprimir los elementos (columnas) de cada fila
 * */
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray) {

    // Escribir código...

}


/**
 * Esta funcion debe recibir como parametro el puntero a la ultima celda, que debe ser la "goal"
 * Con la información del ID_parent y entendiendo que todas las celdas visitadas
 * ya deben estar en el arreglo closed_list, se debe poder buscar la ruta y presentarla en pantalla
 *
 * */
void print_path(Cell_map_t *working_cell){

    // Escribir código...

}

/**
 * == A* algorithm ==
 * Aca es donde se hace toda la magia, todo lo de arriba es necesario, pero
 * el algoritmo se ejecuta es en esta funcion.
 *
 * Esta función es la descripción literal del pseudocodigo...
  * */
void A_star_algorithm(void){

// Escribir código...
}
