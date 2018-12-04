# List of algorithms (CAP 5636)

1.Depth First Search:-

  Depth-first search is a tree-based graph traversal algorithm that is utilized to search through a graph. Unlike BFS, a DFS algorithm crosses a tree or a graph from the parent vertex to its children and grandchildren vertices in a single path until it reaches a deadlock. When there are no more vertices to visit in a path, the algorithm will backtrack to the vertex from which it can take another path, and the whole process goes on again and again until all the vertices have been visited.

2.Breadth First Search:-

  Breadth-first search algorithm crosses a graph in a breadth ward movement and utilizes a queue to make sure that whenever a deadlock occurs, get to the next vertex and start a search. We navigate through a whole dimension of children first, before proceeding onward to cross through the grandchildren and similarly, the algorithm crosses a whole dimension of grandchildren before moving on to the great-grandchildren.

3.Uniform Cost Search:-

    Uniform cost search is a tree searching algorithm identified with the breadth-first search. Uniform cost search decides a way to the goal state that has the lowest weight. The algorithm assumes that all edges weight above equal to or above to 0. On the off chance that any edges have negative weight, it is conceivable that a path p starts with a vertex whose edge to its parent vertex have a high positive weight, which will bar it from consideration by the algorithm.

4.Dijkstra's Algorithm:-

  Dijkstra's algorithm is fundamentally the same as Prim's algorithm for minimum spanning tree. Like Prim's MST, we produce an SPT (shortest path tree) with given source as root. We maintain two sets; one set contains vertices incorporated into the shortest path tree, other set includes vertices not yet incorporated into the shortest path tree. At each step of the algorithm, we discover a vertex which is in the other (set of not yet included) and has a minimum distance from the source.

5.Minimum-Spanning-Tree:-
  A minimum spanning tree (MST) or minimum weight spanning tree is a subset of the edges of an associated, edge-weighted undirected graph that connects all the vertices, with no cycles and with the minimum possible aggregate edge weight.

  5.1 Kruskal's Algorithm:
    Kruskal's algorithm is a minimum-spanning-tree algorithm which finds an edge of the least possible weight that connects any two trees in the forest. It is a greedy algorithm in graph theory as it finds a minimum spanning tree for an associated weighted graph including increasing cost arcs at each step.

  5.2 Prim's Algorithm:
    Prim's algorithm is a greedy algorithm that finds a minimum spanning tree for a weighted undirected graph. This means it finds a subset of the edges that forms a tree that includes each vertex, where the aggregate weight of the considerable number of edges in the tree is minimized.

6.Priority Queues:-

  Priority Queue is similar to a queue where we insert an element from the back and remove an element from the front, yet with one distinction that the reasonable request of elements in the priority queue depends on the priority of the elements. The element with the highest priority will be moved to the front of the queue, and one with the lowest priority will move to the back of the queue.

7.A* Search:-

  A* is a computer algorithm that is broadly used in pathfinding and graph traversal. The algorithm proficiently plots a walkable path between multiple nodes, or points, on the graph. In a map with many obstacles, pathfinding from points A to B can be troublesome. A robot, for instance, without getting many other bearings, will proceed until the point when it encounters an obstacle, as in the path-discovering example to one side underneath. However, the A* algorithm introduces a heuristic into a standard graph-searching algorithm, essentially preparing at each step, so a more optimal decision is made.

8.Minimax

  Minimax is a decision-making algorithm, regularly used in a turn-based, two player games. The objective of the algorithm is to locate the optimal next move. In the algorithm, one player is known as the maximizer, and the other player is a minimizer. In the event that we assign an assessment score to the game board, one player tries to choose a game state with the maximum score, while alternate chooses a state with the minimum score. At the end of the day, the maximizer works to get the highest score, while the minimizer tries to inspire the lowest score by attempting to counter moves.

9.Alpha-Beta Pruning

  Alpha-Beta pruning is not really another algorithm, rather an optimization technique for minimax algorithm. It reduces the computation time by a tremendous factor. This allows us to search much faster and even go into more profound levels in the game tree. It cuts off branches in the game tree which require not be searched because there as of now exists a superior move accessible. It is called Alpha-Beta pruning because it passes 2 additional parameters in the minimax work, namely alpha and beta.

10.Expectimax Search

  The expectiminimax algorithm is a variety of the minimax algorithm, for use in artificial intelligence systems that play two-player zero-sum games, such as backgammon, in which the outcome depends on a combination of the player's skill and chance elements such as bones rolls. Notwithstanding "min" and "max" nodes of the customary minimax tree, this variation has "possibility" nodes, which take the normal estimation of a random occasion happening. In game theory terms, an expectiminimax tree is the game tree of an extensive-form game of impeccable, yet incomplete information.

11.Q-Learning

  Q-learning is a reinforcement learning technique used in machine learning. The objective of Q-learning is to take in a policy, which tells an agent what move to make under what circumstances. It does not require a model of the environment and can handle problems with stochastic transitions and rewards, without requiring adaptations.

12.State Space Graphs

  State space search is a process used in the field of computer science, including artificial intelligence, in which successive configurations or states of an instance are considered, with the goal of finding an objective state with the desired property. Problems are frequently modeled as a state space, a set of states that a problem can be in. The set of states forms a graph where two states are associated if there is a task that can be performed to transform the first state into the second.

13.Iterative Deepening

  The iterative deepening profundity first search is a hybrid of BFS and DFS. In IDDFS, we perform DFS up to a specific "limited profundity," and continue increasing this "limited profundity" after each emphasis. It is a state space/graph search strategy in which a limited profundity version of profundity first search is run more than once with increasing profundity limits until the point when the objective is found. IDDFS is equivalent to broadness first search, yet uses much less memory; at every cycle, it visits the nodes in the search tree in the same request as profundity first search. However, the cumulative request in which nodes are first visited is viably expansiveness first.

14.Greedy Search

  A greedy algorithm is an algorithmic paradigm that follows the problem solving heuristic of making the locally optimal decision at each stage with the expectation of finding a global optimum. In many problems, a greedy strategy does not usually create an optimal solution, but rather nonetheless a greedy heuristic may yield locally optimal solutions that approximate an all-around optimal solution in a reasonable amount of time.

15.Markov Decision Processes

  MDPs are meant to be a straightforward framing of the problem of learning from communication to accomplish an objective. The agent and the environment associate ceaselessly, the agent selecting actions and the environment responding to these actions and presenting new situations to the agent. Formally, an MDP is used to describe an environment for reinforcement learning, where the environment is entirely observable. Almost all RL problems can be formalized as MDPs.

16.Reinforcement Learning

  Reinforcement learning is a region of machine learning worried about how software agents should take actions in an environment so as to maximize some thought of cumulative reward. The problem, because of its simplification, is studied in many different disciplines, such as game theory, control theory, operations research, information theory, simulation-based optimization, multi-agent systems, swarm intelligence, statistics, and genetic algorithms.
