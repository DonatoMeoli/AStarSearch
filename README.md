# A* Search [![Build Status](https://travis-ci.org/DonatoMeoli/AStarSearch.svg?branch=master)](https://travis-ci.org/DonatoMeoli/AStarSearch)

This code is an implementation of the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) developed 
during the Knowledge Engineering course @ 
[Department of Computer Science](http://www.uniba.it/ricerca/dipartimenti/informatica) 
@ [University of Bari "Aldo Moro"](http://www.uniba.it/) under the supervision of Dr. 
[Floriana Esposito](http://lacam.di.uniba.it/people/FlorianaEsposito.html).

## Pseudocode

Pseudocode description of the algorithm from Russell and Norvig's 
["Artificial Intelligence: A Modern Approach"](http://aima.cs.berkeley.edu/).

LRTA\*\-AGENT selects an action according to the values of neighboring states, which are updated as the agent moves 
about the state space.

__function__ LRTA\*\-AGENT(_s'_) __returns__ an action  
&emsp;__inputs__: _s'_, a percept that identifies the current state  
&emsp;__persistent__: _result_, a table, indexed by state and action, initially empty  
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;_H_, a table of cost estimates indexed by state, initially empty  
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;_s_, _a_, the previous state and action, initially null  

&emsp;__if__ GOAL\-TEST(_s'_) __then return__ _stop_  
&emsp;__if__ _s'_ is a new state (not in _H_) __then__ _H_\[_s'_\] &larr; _h_(_s'_)  
&emsp;__if__ _s_ is not null  
&emsp;&emsp;&emsp;_result_\[_s_, _a_\] &larr; _s'_  
&emsp;&emsp;&emsp;_H_\[_s_\] &larr; &emsp;__min__&emsp; LRTA\*\-COST(_s_, _b_, _result_\[_s_, _b_\], _H_)  
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;<sub>_b_ &Element; ACTIONS(_s_)</sub>  
&emsp;_a_ &larr; an action _b_ in ACTIONS(_s'_) that minimizes LRTA\*\-COST(_s'_, _b_, _result_\[_s'_, _b_\], _H_)  
&emsp;_s_ &larr; _s'_  
&emsp;__return__ a

__function__ LRTA\*\-COST(_s_, _a_, _s'_, _H_) __returns__ a cost estimate  
&emsp;__if__ _s'_ is undefined __then return__ _h_(_s_)  
&emsp;__else return__ c(_s_, _a_, _s'_) + _H_\[_s'_\]  

## Running the software

To compile the source code and run the software you just type into the terminal:

```
$ make
$ ./FindPath.o
$ ./8Puzzle.o {134862705|281043765|281463075|567408321|etc.}
$ ./MinPathToBucharest.o {Arad|Bucharest|Craiova|Drobeta|Eforie|Fagaras|Giurgiu|Hirsova|Iasi|Lugoj|Mehadia|Neamt|Oradea|Pitesti|RimnicuVilcea|Sibiu|Timisoara|Urziceni|Vaslui|Zerind}
```

## License [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This software is released under the MIT License. See the [LICENSE](LICENSE) file for details.