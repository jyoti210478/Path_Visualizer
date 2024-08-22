$(document).ready(function(){
    var lang = "c";
    var algo = "b";
    var cCode = {
        "Bubble": `
            <div>// C program for implementation of BFS</div>
            <div>#include &lt;stdio.h&gt;</div>
            <div>#include &lt;stdbool.h&gt;</div>
            <div>// A function to implement bubble sort></div>
            <div>// BFS algorithm in C</div>
            <div>#include &lt;stdio.h&gt;</div>
            <div>#include &lt;stdbool.h&gt;</div>
            <div>#include &lt;stdlib.h&gt;</div>
            <div>#define MAX 100</div>
            <div>typedef struct {</div>
            <div>    int front, rear;</div>
            <div>    int items[MAX];</div>
            <div>} Queue;</div>
            <div>void initQueue(Queue* q) {</div>
            <div>    q->front = -1;</div>
            <div>    q->rear = -1;</div>
            <div>}</div>
            <div>bool isEmpty(Queue* q) {</div>
            <div>    return q->rear == -1;</div>
            <div>}</div>
            <div>void enqueue(Queue* q, int value) {</div>
            <div>    if (q->rear == MAX - 1) {</div>
            <div>        printf("Queue is full\\n");</div>
            <div>        return;</div>
            <div>    }</div>
            <div>    if (isEmpty(q)) {</div>
            <div>        q->front = 0;</div>
            <div>    }</div>
            <div>    q->items[++q->rear] = value;</div>
            <div>}</div>
            <div>int dequeue(Queue* q) {</div>
            <div>    if (isEmpty(q)) {</div>
            <div>        printf("Queue is empty\\n");</div>
            <div>        return -1;</div>
            <div>    }</div>
            <div>    return q->items[q->front++];</div>
            <div>}</div>
            <div>void BFS(int start, int graph[MAX][MAX], int n) {</div>
            <div>    Queue q;</div>
            <div>    initQueue(&q);</div>
            <div>    bool visited[MAX] = { false }; </div>
            <div>    enqueue(&q, start);</div>
            <div>    visited[start] = true;</div>
            <div>    while (!isEmpty(&q)) {</div>
            <div>        int node = dequeue(&q);</div>
            <div>        printf("%d ", node);</div>
            <div>        for (int i = 0; i &lt; n; i++) {</div>
            <div>            if (graph[node][i] && !visited[i]) {</div>
            <div>                visited[i] = true;</div>
            <div>                enqueue(&q, i);</div>
            <div>            }</div>
            <div>        }</div>
            <div>    }</div>
            <div>}</div>`,
        "DFS": `
<div>// C program for implementation of DFS</div>
<div>#include &lt;stdio.h&gt;</div>
<div>#include &lt;stdbool.h&gt;</div>
<div>#define MAX 100</div>
<div>void DFSUtil(int node, bool visited[], int graph[MAX][MAX], int n) {</div>
<div>    visited[node] = true;</div>
<div>    printf("%d ", node);</div>
<div>    for (int i = 0; i &lt; n; i++) {</div>
<div>        if (graph[node][i] && !visited[i]) {</div>
<div>            DFSUtil(i, visited, graph, n);</div>
<div>        }</div>
<div>    }</div>
<div>}</div>
<div>void DFS(int start, int graph[MAX][MAX], int n) {</div>
<div>    bool visited[MAX] = { false };</div>
<div>    DFSUtil(start, visited, graph, n);</div>
<div>}</div>
<div>// Driver program to test the above function</div>
<div>int main() {</div>
<div>    int graph[MAX][MAX] = { /* Your graph data here */ };</div>
<div>    int n = 5; // Number of vertices</div>
<div>    int start = 0; // Starting vertex</div>
<div>    DFS(start, graph, n);</div>
<div>    return 0;</div>
<div>}</div>`,

        
"DIJKSTRA": `
<div>// Dijkstra's algorithm in C</div>
<div>#include &lt;stdio.h&gt;</div>
<div>#include &lt;stdbool.h&gt;</div>
<div>#include &lt;limits.h&gt;</div>
<div>#define MAX 100</div>
<div>#define INF INT_MAX</div>
<div>// Function to find the vertex with the minimum distance value</div>
<div>int minDistance(int dist[], bool visited[], int n) {</div>
<div>    int min = INF, min_index;</div>
<div>    for (int v = 0; v &lt; n; v++) {</div>
<div>        if (!visited[v] &amp;&amp; dist[v] &lt;= min) {</div>
<div>            min = dist[v];</div>
<div>            min_index = v;</div>
<div>        }</div>
<div>    }</div>
<div>    return min_index;</div>
<div>}</div>
<div>// Function to print the distance array</div>
<div>void printSolution(int dist[], int n) {</div>
<div>    printf("Vertex   Distance from Source\\n");</div>
<div>    for (int i = 0; i &lt; n; i++) {</div>
<div>        printf("%d \\t %d\\n", i, dist[i]);</div>
<div>    }</div>
<div>}</div>
<div>// Dijkstra's algorithm implementation</div>
<div>void dijkstra(int graph[MAX][MAX], int src, int n) {</div>
<div>    int dist[MAX]; // Output array. dist[i] will hold the shortest distance from src to i</div>
<div>    bool visited[MAX]; // visited[i] will be true if vertex i is included in the shortest path tree</div>
<div>    for (int i = 0; i &lt; n; i++) {</div>
<div>        dist[i] = INF;</div>
<div>        visited[i] = false;</div>
<div>    }</div>
<div>    dist[src] = 0; // Distance of source vertex from itself is always 0</div>
<div>    for (int count = 0; count &lt; n - 1; count++) {</div>
<div>        int u = minDistance(dist, visited, n);</div>
<div>        visited[u] = true;</div>
<div>        for (int v = 0; v &lt; n; v++) {</div>
<div>            if (!visited[v] &amp;&amp; graph[u][v] &amp;&amp; dist[u] != INF &amp;&amp; dist[u] + graph[u][v] &lt; dist[v]) {</div>
<div>                dist[v] = dist[u] + graph[u][v];</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>    printSolution(dist, n);</div>
<div>}</div>
<div>// Driver program to test the above function</div>
<div>int main() {</div>
<div>    int graph[MAX][MAX] = { /* Your graph data here */ };</div>
<div>    int n = 5; // Number of vertices</div>
<div>    int src = 0; // Source vertex</div>
<div>    dijkstra(graph, src, n);</div>
<div>    return 0;</div>
<div>}</div> `,

"A* ALGO": `
<div>// A* algorithm in C</div>
<div>#include &lt;stdio.h&gt;</div>
<div>#include &lt;stdbool.h&gt;</div>
<div>#include &lt;limits.h&gt;</div>
<div>#include &lt;math.h&gt;</div>
<div>#define MAX 100</div>
<div>#define INF INT_MAX</div>
<div>typedef struct {</div>
<div>    int x, y;</div>
<div>} Point;</div>
<div>typedef struct {</div>
<div>    int parent; // Index of the parent node in the path</div>
<div>    int g; // Cost from the start node to this node</div>
<div>    int h; // Heuristic cost estimate from this node to the goal</div>
<div>    int f; // Total cost (g + h)</div>
<div>} Node;</div>
<div>int heuristic(Point a, Point b) {</div>
<div>    return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan distance</div>
<div>}</div>
<div>void AStar(Point start, Point goal, int graph[MAX][MAX], int n, int m) {</div>
<div>    Node nodes[MAX][MAX];</div>
<div>    bool closedList[MAX][MAX] = { false };</div>
<div>    bool openList[MAX][MAX] = { false };</div>
<div>    int dx[] = { 0, 1, 0, -1 }; // Directions for moving in the grid</div>
<div>    int dy[] = { 1, 0, -1, 0 };</div>
<div>    int startX = start.x, startY = start.y;</div>
<div>    int goalX = goal.x, goalY = goal.y;</div>
<div>    for (int i = 0; i &lt; n; i++) {</div>
<div>        for (int j = 0; j &lt; m; j++) {</div>
<div>            nodes[i][j].g = INF;</div>
<div>            nodes[i][j].h = INF;</div>
<div>            nodes[i][j].f = INF;</div>
<div>            nodes[i][j].parent = -1;</div>
<div>        }</div>
<div>    }</div>
<div>    nodes[startX][startY].g = 0;</div>
<div>    nodes[startX][startY].h = heuristic(start, goal);</div>
<div>    nodes[startX][startY].f = nodes[startX][startY].h;</div>
<div>    openList[startX][startY] = true;</div>
<div>    while (true) {</div>
<div>        int x = -1, y = -1;</div>
<div>        int minF = INF;</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            for (int j = 0; j &lt; m; j++) {</div>
<div>                if (openList[i][j] &amp;&amp; nodes[i][j].f &lt; minF) {</div>
<div>                    minF = nodes[i][j].f;</div>
<div>                    x = i;</div>
<div>                    y = j;</div>
<div>                }</div>
<div>            }</div>
<div>        }</div>
<div>        if (x == -1 || y == -1) break;</div>
<div>        if (x == goalX &amp;&amp; y == goalY) {</div>
<div>            printf("Path found!\\n");</div>
<div>            break;</div>
<div>        }</div>
<div>        openList[x][y] = false;</div>
<div>        closedList[x][y] = true;</div>
<div>        for (int d = 0; d &lt; 4; d++) {</div>
<div>            int newX = x + dx[d];</div>
<div>            int newY = y + dy[d];</div>
<div>            if (newX &gt;= 0 &amp;&amp; newX &lt; n &amp;&amp; newY &gt;= 0 &amp;&amp; newY &lt; m &amp;&amp; !closedList[newX][newY] &amp;&amp; graph[newX][newY] == 0) {</div>
<div>                int newG = nodes[x][y].g + 1;</div>
<div>                int newH = heuristic((Point){ newX, newY }, goal);</div>
<div>                int newF = newG + newH;</div>
<div>                if (!openList[newX][newY] || newF &lt; nodes[newX][newY].f) {</div>
<div>                    nodes[newX][newY].g = newG;</div>
<div>                    nodes[newX][newY].h = newH;</div>
<div>                    nodes[newX][newY].f = newF;</div>
<div>                    nodes[newX][newY].parent = x * m + y;</div>
<div>                    openList[newX][newY] = true;</div>
<div>                }</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>// Optionally, you can backtrack from goal to start to print the path</div>
<div>}</div>
<div>// Driver program to test the above function</div>
<div>int main() {</div>
<div>    int graph[MAX][MAX] = { /* Your grid data here */ };</div>
<div>    int n = 5; // Number of rows</div>
<div>    int m = 5; // Number of columns</div>
<div>    Point start = { 0, 0 }; // Starting point</div>
<div>    Point goal = { 4, 4 }; // Goal point</div>
<div>    AStar(start, goal, graph, n, m);</div>
<div>    return 0;</div>
<div>}</div>
      `,

        


    }
    

    
    var cppCode = {
        "BFS": `
<div>// C++ program for implementation of BFS</div>
<div>#include &lt;iostream&gt;</div>
<div>#include &lt;queue&gt;</div>
<div>#include &lt;vector&gt;</div>
<div>#include &lt;list&gt;</div>
<div>#include &lt;climits&gt;</div>
<div>#define MAX 100</div>
<div>using namespace std;</div>
<div>void BFS(int start, vector&lt;int&gt; graph[MAX], int n) {</div>
<div>    vector&lt;bool&gt; visited(n, false);</div>
<div>    queue&lt;int&gt; q;</div>
<div>    q.push(start);</div>
<div>    visited[start] = true;</div>
<div>    while (!q.empty()) {</div>
<div>        int node = q.front();</div>
<div>        q.pop();</div>
<div>        cout &lt;&lt; node &lt;&lt; " ";</div>
<div>        for (int neighbor : graph[node]) {</div>
<div>            if (!visited[neighbor]) {</div>
<div>                visited[neighbor] = true;</div>
<div>                q.push(neighbor);</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>}</div>
<div>// Driver program to test the above function</div>
<div>int main() {</div>
<div>    vector&lt;int&gt; graph[MAX];</div>
<div>    int n = 5; // Number of vertices</div>
<div>    // Example graph data</div>
<div>    graph[0].push_back(1);</div>
<div>    graph[0].push_back(2);</div>
<div>    graph[1].push_back(3);</div>
<div>    graph[2].push_back(3);</div>
<div>    graph[3].push_back(4);</div>
<div>    int start = 0; // Starting vertex</div>
<div>    BFS(start, graph, n);</div>
<div>    return 0;</div>
<div>}</div>
  `,

  "DFS": `
  <div>// C++ program for implementation of DFS with user input</div>
<div>#include &lt;iostream&gt;</div>
<div>#include &lt;vector&gt;</div>
<div>#include &lt;stack&gt;</div>
<div>#include &lt;algorithm&gt;</div>
<div>#define MAX 100</div>
<div>using namespace std;</div>
<div>void DFS(int start, vector&lt;int&gt; graph[MAX], vector&lt;bool&gt; &amp;visited) {</div>
<div>    stack&lt;int&gt; s;</div>
<div>    s.push(start);</div>
<div>    visited[start] = true;</div>
<div>    while (!s.empty()) {</div>
<div>        int node = s.top();</div>
<div>        s.pop();</div>
<div>        cout &lt;&lt; node &lt;&lt; " ";</div>
<div>        for (int i = graph[node].size() - 1; i &gt;= 0; --i) {</div>
<div>            int neighbor = graph[node][i];</div>
<div>            if (!visited[neighbor]) {</div>
<div>                visited[neighbor] = true;</div>
<div>                s.push(neighbor);</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>}</div>
<div>int main() {</div>
<div>    int n, m;</div>
<div>    cout &lt;&lt; "Enter number of vertices: ";</div>
<div>    cin &gt;&gt; n;</div>
<div>    cout &lt;&lt; "Enter number of edges: ";</div>
<div>    cin &gt;&gt; m;</div>
<div>    vector&lt;int&gt; graph[MAX];</div>
<div>    cout &lt;&lt; "Enter edges (u v) where u and v are 0-based vertex indices:" &lt;&lt; endl;</div>
<div>    for (int i = 0; i &lt; m; ++i) {</div>
<div>        int u, v;</div>
<div>        cin &gt;&gt; u &gt;&gt; v;</div>
<div>        graph[u].push_back(v);</div>
<div>        graph[v].push_back(u); // Uncomment if the graph is undirected</div>
<div>    }</div>
<div>    vector&lt;bool&gt; visited(n, false);</div>
<div>    int start;</div>
<div>    cout &lt;&lt; "Enter starting vertex: ";</div>
<div>    cin &gt;&gt; start;</div>
<div>    cout &lt;&lt; "DFS traversal starting from vertex " &lt;&lt; start &lt;&lt; ":" &lt;&lt; endl;</div>
<div>    DFS(start, graph, visited);</div>
<div>    return 0;</div>
<div>}</div>

    `,


    "DIJKSTAR": `
<div>// C++ program for implementation of Dijkstra's Algorithm with user input</div>
<div>#include &lt;iostream&gt;</div>
<div>#include &lt;vector&gt;</div>
<div>#include &lt;queue&gt;</div>
<div>#include &lt;climits&gt;</div>
<div>#define MAX 100</div>
<div>using namespace std;</div>
<div>void Dijkstra(int start, vector&lt;pair&lt;int, int&gt;&gt; graph[MAX], int n) {</div>
<div>    vector&lt;int&gt; dist(n, INT_MAX);</div>
<div>    priority_queue&lt;pair&lt;int, int&gt;, vector&lt;pair&lt;int, int&gt;&gt;, greater&lt;pair&lt;int, int&gt;&gt; pq;</div>
<div>    dist[start] = 0;</div>
<div>    pq.push(make_pair(0, start));</div>
<div>    while (!pq.empty()) {</div>
<div>        int u = pq.top().second;</div>
<div>        int d = pq.top().first;</div>
<div>        pq.pop();</div>
<div>        if (d &gt; dist[u]) continue;</div>
<div>        for (const auto &amp;neighbor : graph[u]) {</div>
<div>            int v = neighbor.first;</div>
<div>            int weight = neighbor.second;</div>
<div>            if (dist[u] + weight &lt; dist[v]) {</div>
<div>                dist[v] = dist[u] + weight;</div>
<div>                pq.push(make_pair(dist[v], v));</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>    cout &lt;&lt; "Distances from source vertex " &lt;&lt; start &lt;&lt; ":" &lt;&lt; endl;</div>
<div>    for (int i = 0; i &lt; n; ++i) {</div>
<div>        if (dist[i] == INT_MAX) {</div>
<div>            cout &lt;&lt; "Vertex " &lt;&lt; i &lt;&lt; ": INF" &lt;&lt; endl;</div>
<div>        } else {</div>
<div>            cout &lt;&lt; "Vertex " &lt;&lt; i &lt;&lt; ": " &lt;&lt; dist[i] &lt;&lt; endl;</div>
<div>        }</div>
<div>    }</div>
<div>}</div>
<div>int main() {</div>
<div>    int n, m;</div>
<div>    cout &lt;&lt; "Enter number of vertices: ";</div>
<div>    cin &gt;&gt; n;</div>
<div>    cout &lt;&lt; "Enter number of edges: ";</div>
<div>    cin &gt;&gt; m;</div>
<div>    vector&lt;pair&lt;int, int&gt;&gt; graph[MAX];</div>
<div>    cout &lt;&lt; "Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:" &lt;&lt; endl;</div>
<div>    for (int i = 0; i &lt; m; ++i) {</div>
<div>        int u, v, w;</div>
<div>        cin &gt;&gt; u &gt;&gt; v &gt;&gt; w;</div>
<div>        graph[u].push_back(make_pair(v, w));</div>
<div>        // graph[v].push_back(make_pair(u, w)); // Uncomment if the graph is undirected</div>
<div>    }</div>
<div>    int start;</div>
<div>    cout &lt;&lt; "Enter starting vertex: ";</div>
<div>    cin &gt;&gt; start;</div>
<div>    Dijkstra(start, graph, n);</div>
<div>    return 0;</div>
<div>}</div>

      `,

      
      "A* ALGO": `
<div>// C++ program for implementation of A* Algorithm with user input</div>
<div>#include &lt;iostream&gt;</div>
<div>#include &lt;vector&gt;</div>
<div>#include &lt;queue&gt;</div>
<div>#include &lt;cmath&gt;</div>
<div>#include &lt;limits&gt;</div>
<div>#define MAX 100</div>
<div>using namespace std;</div>
<div>typedef pair&lt;int, int&gt; pii;</div>
<div>struct Node {</div>
<div>    int vertex;</div>
<div>    int cost;</div>
<div>    int heuristic;</div>
<div>    bool operator &lt;(const Node &amp; other) const {</div>
<div>        return (cost + heuristic) &gt; (other.cost + other.heuristic);</div>
<div>    }</div>
<div>};</div>
<div>vector&lt;pii&gt; graph[MAX];</div>
<div>void AStar(int start, int goal, int n) {</div>
<div>    vector&lt;int&gt; dist(n, numeric_limits&lt;int&gt;::max());</div>
<div>    vector&lt;int&gt; heuristic(n, 0);</div>
<div>    priority_queue&lt;Node&gt; pq;</div>
<div>    pq.push({start, 0, heuristic[start]});</div>
<div>    dist[start] = 0;</div>
<div>    while (!pq.empty()) {</div>
<div>        int u = pq.top().vertex;</div>
<div>        int cost = pq.top().cost;</div>
<div>        pq.pop();</div>
<div>        if (u == goal) {</div>
<div>            cout &lt;&lt; "Path cost to goal: " &lt;&lt; cost &lt;&lt; endl;</div>
<div>            return;</div>
<div>        }</div>
<div>        for (const auto &amp;neighbor : graph[u]) {</div>
<div>            int v = neighbor.first;</div>
<div>            int weight = neighbor.second;</div>
<div>            int newDist = cost + weight;</div>
<div>            if (newDist &lt; dist[v]) {</div>
<div>                dist[v] = newDist;</div>
<div>                pq.push({v, newDist, heuristic[v]});</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>    cout &lt;&lt; "Goal not reachable." &lt;&lt; endl;</div>
<div>}</div>
<div>int main() {</div>
<div>    int n, m;</div>
<div>    cout &lt;&lt; "Enter number of vertices: ";</div>
<div>    cin &gt;&gt; n;</div>
<div>    cout &lt;&lt; "Enter number of edges: ";</div>
<div>    cin &gt;&gt; m;</div>
<div>    cout &lt;&lt; "Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:" &lt;&lt; endl;</div>
<div>    for (int i = 0; i &lt; m; ++i) {</div>
<div>        int u, v, w;</div>
<div>        cin &gt;&gt; u &gt;&gt; v &gt;&gt; w;</div>
<div>        graph[u].push_back(make_pair(v, w));</div>
<div>        graph[v].push_back(make_pair(u, w)); // Uncomment if the graph is undirected</div>
<div>    }</div>
<div>    vector&lt;int&gt; heuristic(n);</div>
<div>    cout &lt;&lt; "Enter heuristic values for each vertex:" &lt;&lt; endl;</div>
<div>    for (int i = 0; i &lt; n; ++i) {</div>
<div>        cin &gt;&gt; heuristic[i];</div>
<div>    }</div>
<div>    int start, goal;</div>
<div>    cout &lt;&lt; "Enter starting vertex: ";</div>
<div>    cin &gt;&gt; start;</div>
<div>    cout &lt;&lt; "Enter goal vertex: ";</div>
<div>    cin &gt;&gt; goal;</div>
<div>    AStar(start, goal, n);</div>
<div>    return 0;</div>
<div>}</div>

        `,
    
    }

    var javaCode = {
        "BFS": `
       <div>// Java program for implementation of BFS with user input</div>
<div>import java.util.*;</div>
<div>public class BFSExample {</div>
<div>    static final int MAX = 100;</div>
<div>    static List&lt;Integer&gt;[] graph = new ArrayList[MAX];</div>
<div>    static boolean[] visited = new boolean[MAX];</div>
<div>    static void BFS(int start, int n) {</div>
<div>        Queue&lt;Integer&gt; queue = new LinkedList&lt;&gt;();</div>
<div>        queue.add(start);</div>
<div>        visited[start] = true;</div>
<div>        while (!queue.isEmpty()) {</div>
<div>            int node = queue.poll();</div>
<div>            System.out.print(node + " ");</div>
<div>            for (int neighbor : graph[node]) {</div>
<div>                if (!visited[neighbor]) {</div>
<div>                    visited[neighbor] = true;</div>
<div>                    queue.add(neighbor);</div>
<div>                }</div>
<div>            }</div>
<div>        }</div>
<div>        System.out.println();</div>
<div>    }</div>
<div>    public static void main(String[] args) {</div>
<div>        Scanner sc = new Scanner(System.in);</div>
<div>        System.out.print("Enter number of vertices: ");</div>
<div>        int n = sc.nextInt();</div>
<div>        System.out.print("Enter number of edges: ");</div>
<div>        int m = sc.nextInt();</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            graph[i] = new ArrayList&lt;&gt;();</div>
<div>        }</div>
<div>        System.out.println("Enter edges (u v) where u and v are 0-based vertex indices:");</div>
<div>        for (int i = 0; i &lt; m; i++) {</div>
<div>            int u = sc.nextInt();</div>
<div>            int v = sc.nextInt();</div>
<div>            graph[u].add(v);</div>
<div>            graph[v].add(u); // Uncomment if the graph is undirected</div>
<div>        }</div>
<div>        System.out.print("Enter starting vertex: ");</div>
<div>        int start = sc.nextInt();</div>
<div>        System.out.println("BFS traversal starting from vertex " + start + ":");</div>
<div>        BFS(start, n);</div>
<div>        sc.close();</div>
<div>    }</div>
<div>}</div>
`,
        

        "DFS": `
<div>// Java program for implementation of DFS with user input</div>
<div>import java.util.*;</div>
<div>public class DFSExample {</div>
<div>    static final int MAX = 100;</div>
<div>    static List&lt;Integer&gt;[] graph = new ArrayList[MAX];</div>
<div>    static boolean[] visited = new boolean[MAX];</div>
<div>    static void DFS(int node) {</div>
<div>        System.out.print(node + " ");</div>
<div>        visited[node] = true;</div>
<div>        for (int neighbor : graph[node]) {</div>
<div>            if (!visited[neighbor]) {</div>
<div>                DFS(neighbor);</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>    public static void main(String[] args) {</div>
<div>        Scanner sc = new Scanner(System.in);</div>
<div>        System.out.print("Enter number of vertices: ");</div>
<div>        int n = sc.nextInt();</div>
<div>        System.out.print("Enter number of edges: ");</div>
<div>        int m = sc.nextInt();</div>
<div>        // Initialize graph</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            graph[i] = new ArrayList&lt;&gt;();</div>
<div>        }</div>
<div>        System.out.println("Enter edges (u v) where u and v are 0-based vertex indices:");</div>
<div>        for (int i = 0; i &lt; m; i++) {</div>
<div>            int u = sc.nextInt();</div>
<div>            int v = sc.nextInt();</div>
<div>            graph[u].add(v);</div>
<div>            graph[v].add(u); // Uncomment if the graph is undirected</div>
<div>        }</div>
<div>        // Reset visited array</div>
<div>        Arrays.fill(visited, false);</div>
<div>        System.out.print("Enter starting vertex: ");</div>
<div>        int start = sc.nextInt();</div>
<div>        System.out.println("DFS traversal starting from vertex " + start + ":");</div>
<div>        DFS(start);</div>
<div>        sc.close();</div>
<div>    }</div>
<div>}</div>
`,
        

        "DIJKSTRA": `
<div>// Java program for implementation of Dijkstra's Algorithm with user input</div>
<div>import java.util.*;</div>
<div>public class DijkstraExample {</div>
<div>    static final int MAX = 100;</div>
<div>    static List&lt;int[]&gt;[] graph = new ArrayList[MAX];</div>
<div>    static int[] dist = new int[MAX];</div>
<div>    static final int INF = Integer.MAX_VALUE;</div>
<div>    static void dijkstra(int start, int n) {</div>
<div>        PriorityQueue&lt;int[]&gt; pq = new PriorityQueue&lt;&gt;(Comparator.comparingInt(a -&gt; a[1]));</div>
<div>        Arrays.fill(dist, INF);</div>
<div>        dist[start] = 0;</div>
<div>        pq.add(new int[]{start, 0});</div>
<div>        while (!pq.isEmpty()) {</div>
<div>            int[] current = pq.poll();</div>
<div>            int u = current[0];</div>
<div>            int currentDist = current[1];</div>
<div>            if (currentDist &gt; dist[u]) continue;</div>
<div>            for (int[] neighbor : graph[u]) {</div>
<div>                int v = neighbor[0];</div>
<div>                int weight = neighbor[1];</div>
<div>                int newDist = dist[u] + weight;</div>
<div>                if (newDist &lt; dist[v]) {</div>
<div>                    dist[v] = newDist;</div>
<div>                    pq.add(new int[]{v, newDist});</div>
<div>                }</div>
<div>            }</div>
<div>        }</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            if (dist[i] == INF) {</div>
<div>                System.out.println("Distance to vertex " + i + " is: INF");</div>
<div>            } else {</div>
<div>                System.out.println("Distance to vertex " + i + " is: " + dist[i]);</div>
<div>            }</div>
<div>        }</div>
<div>    }</div>
<div>    public static void main(String[] args) {</div>
<div>        Scanner sc = new Scanner(System.in);</div>
<div>        System.out.print("Enter number of vertices: ");</div>
<div>        int n = sc.nextInt();</div>
<div>        System.out.print("Enter number of edges: ");</div>
<div>        int m = sc.nextInt();</div>
<div>        // Initialize graph</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            graph[i] = new ArrayList&lt;&gt;();</div>
<div>        }</div>
<div>        System.out.println("Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:");</div>
<div>        for (int i = 0; i &lt; m; i++) {</div>
<div>            int u = sc.nextInt();</div>
<div>            int v = sc.nextInt();</div>
<div>            int w = sc.nextInt();</div>
<div>            graph[u].add(new int[]{v, w});</div>
<div>            // graph[v].add(new int[]{u, w}); // Uncomment if the graph is undirected</div>
<div>        }</div>
<div>        System.out.print("Enter starting vertex: ");</div>
<div>        int start = sc.nextInt();</div>
<div>        System.out.println("Dijkstra's shortest path from vertex " + start + ":");</div>
<div>        dijkstra(start, n);</div>
<div>        sc.close();</div>
<div>    }</div>
<div>}</div>
`,

        


        "A* ALGO": `
<div>// Java program for implementation of A* Algorithm with user input</div>
<div>import java.util.*;</div>
<div>public class AStarExample {</div>
<div>    static final int MAX = 100;</div>
<div>    static List&lt;int[]&gt;[] graph = new ArrayList[MAX];</div>
<div>    static int[] heuristic = new int[MAX];</div>
<div>    static int[] dist = new int[MAX];</div>
<div>    static final int INF = Integer.MAX_VALUE;</div>
<div>    static class Node {</div>
<div>        int vertex, cost;</div>
<div>        Node(int vertex, int cost) {</div>
<div>            this.vertex = vertex;</div>
<div>            this.cost = cost;</div>
<div>        }</div>
<div>    }</div>
<div>    static void aStar(int start, int goal, int n) {</div>
<div>        PriorityQueue&lt;Node&gt; pq = new PriorityQueue&lt;&gt;(Comparator.comparingInt(a -&gt; a.cost));</div>
<div>        Arrays.fill(dist, INF);</div>
<div>        Arrays.fill(heuristic, INF);</div>
<div>        dist[start] = 0;</div>
<div>        heuristic[start] = heuristic[start] + heuristic[start]; // Update heuristic cost</div>
<div>        pq.add(new Node(start, heuristic[start]));</div>
<div>        while (!pq.isEmpty()) {</div>
<div>            Node current = pq.poll();</div>
<div>            int u = current.vertex;</div>
<div>            int currentCost = current.cost;</div>
<div>            if (u == goal) {</div>
<div>                System.out.println("Path found with cost: " + dist[goal]);</div>
<div>                return;</div>
<div>            }</div>
<div>            if (currentCost &gt; dist[u]) continue;</div>
<div>            for (int[] neighbor : graph[u]) {</div>
<div>                int v = neighbor[0];</div>
<div>                int weight = neighbor[1];</div>
<div>                int newDist = dist[u] + weight;</div>
<div>                if (newDist &lt; dist[v]) {</div>
<div>                    dist[v] = newDist;</div>
<div>                    int estimatedCost = newDist + heuristic[v];</div>
<div>                    pq.add(new Node(v, estimatedCost));</div>
<div>                }</div>
<div>            }</div>
<div>        }</div>
<div>        System.out.println("No path found.");</div>
<div>    }</div>
<div>    public static void main(String[] args) {</div>
<div>        Scanner sc = new Scanner(System.in);</div>
<div>        System.out.print("Enter number of vertices: ");</div>
<div>        int n = sc.nextInt();</div>
<div>        System.out.print("Enter number of edges: ");</div>
<div>        int m = sc.nextInt();</div>
<div>        // Initialize graph</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            graph[i] = new ArrayList&lt;&gt;();</div>
<div>        }</div>
<div>        System.out.println("Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:");</div>
<div>        for (int i = 0; i &lt; m; i++) {</div>
<div>            int u = sc.nextInt();</div>
<div>            int v = sc.nextInt();</div>
<div>            int w = sc.nextInt();</div>
<div>            graph[u].add(new int[]{v, w});</div>
<div>            // graph[v].add(new int[]{u, w}); // Uncomment if the graph is undirected</div>
<div>        }</div>
<div>        System.out.println("Enter heuristic values for each vertex:");</div>
<div>        for (int i = 0; i &lt; n; i++) {</div>
<div>            heuristic[i] = sc.nextInt();</div>
<div>        }</div>
<div>        System.out.print("Enter starting vertex: ");</div>
<div>        int start = sc.nextInt();</div>
<div>        System.out.print("Enter goal vertex: ");</div>
<div>        int goal = sc.nextInt();</div>
<div>        System.out.println("A* search from vertex " + start + " to vertex " + goal + ":");</div>
<div>        aStar(start, goal, n);</div>
<div>        sc.close();</div>
<div>    }</div>
<div>}</div>
`,

    }

    var pyCode = {
        "BFS": `
<div># Python program for implementation of BFS with user input</div>
<div>from collections import deque</div>
<div>def bfs(start, graph, n):</div>
<div>    visited = [False] * n</div>
<div>    queue = deque([start])</div>
<div>    visited[start] = True</div>
<div>    print("BFS traversal starting from vertex", start, ":")</div>
<div>    while queue:</div>
<div>        node = queue.popleft()</div>
<div>        print(node, end=" ")</div>
<div>        for neighbor in graph[node]:</div>
<div>            if not visited[neighbor]:</div>
<div>                visited[neighbor] = True</div>
<div>                queue.append(neighbor)</div>
<div>    print()</div>
<div>def main():</div>
<div>    n = int(input("Enter number of vertices: "))</div>
<div>    m = int(input("Enter number of edges: "))</div>
<div>    graph = [[] for _ in range(n)]</div>
<div>    print("Enter edges (u v) where u and v are 0-based vertex indices:")</div>
<div>    for _ in range(m):</div>
<div>        u, v = map(int, input().split())</div>
<div>        graph[u].append(v)</div>
<div>        # graph[v].append(u) # Uncomment if the graph is undirected</div>
<div>    start = int(input("Enter starting vertex: "))</div>
<div>    bfs(start, graph, n)</div>
<div>if __name__ == "__main__":</div>
<div>    main()</div>
`,

"DFS": `
<div># Python program for implementation of DFS with user input</div>
<div>def dfs(v, graph, visited):</div>
<div>    visited[v] = True</div>
<div>    print(v, end=" ")</div>
<div>    for neighbor in graph[v]:</div>
<div>        if not visited[neighbor]:</div>
<div>            dfs(neighbor, graph, visited)</div>
<div>def main():</div>
<div>    n = int(input("Enter number of vertices: "))</div>
<div>    m = int(input("Enter number of edges: "))</div>
<div>    graph = [[] for _ in range(n)]</div>
<div>    print("Enter edges (u v) where u and v are 0-based vertex indices:")</div>
<div>    for _ in range(m):</div>
<div>        u, v = map(int, input().split())</div>
<div>        graph[u].append(v)</div>
<div>        # graph[v].append(u) # Uncomment if the graph is undirected</div>
<div>    start = int(input("Enter starting vertex: "))</div>
<div>    visited = [False] * n</div>
<div>    print("DFS traversal starting from vertex", start, ":")</div>
<div>    dfs(start, graph, visited)</div>
<div>if __name__ == "__main__":</div>
<div>    main()</div>
`,

"DIJKSTRA": `
<div># Python program for implementation of Dijkstra's Algorithm with user input</div>
<div>import heapq</div>
<div>def dijkstra(start, graph, n):</div>
<div>    dist = [float('inf')] * n</div>
<div>    dist[start] = 0</div>
<div>    pq = [(0, start)]  # (distance, vertex)</div>
<div>    while pq:</div>
<div>        current_dist, u = heapq.heappop(pq)</div>
<div>        if current_dist &gt; dist[u]:</div>
<div>            continue</div>
<div>        for v, weight in graph[u]:</div>
<div>            distance = current_dist + weight</div>
<div>            if distance &lt; dist[v]:</div>
<div>                dist[v] = distance</div>
<div>                heapq.heappush(pq, (distance, v))</div>
<div>    return dist</div>
<div>def main():</div>
<div>    n = int(input("Enter number of vertices: "))</div>
<div>    m = int(input("Enter number of edges: "))</div>
<div>    graph = [[] for _ in range(n)]</div>
<div>    print("Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:")</div>
<div>    for _ in range(m):</div>
<div>        u, v, w = map(int, input().split())</div>
<div>        graph[u].append((v, w))</div>
<div>        # graph[v].append((u, w)) # Uncomment if the graph is undirected</div>
<div>    start = int(input("Enter starting vertex: "))</div>
<div>    dist = dijkstra(start, graph, n)</div>
<div>    print("Dijkstra's shortest path from vertex", start, ":")</div>
<div>    for i in range(n):</div>
<div>        if dist[i] == float('inf'):</div>
<div>            print(f"Distance to vertex {i} is: INF")</div>
<div>        else:</div>
<div>            print(f"Distance to vertex {i} is: {dist[i]}")</div>
<div>if __name__ == "__main__":</div>
<div>    main()</div>
`,


"A* ALGO": `
<div># Python program for implementation of A* Algorithm with user input</div>
<div>import heapq</div>
<div>def a_star(start, goal, graph, heuristic, n):</div>
<div>    dist = [float('inf')] * n</div>
<div>    dist[start] = 0</div>
<div>    pq = [(heuristic[start], start)]  # (estimated cost, vertex)</div>
<div>    came_from = [-1] * n  # To reconstruct path</div>
<div>    while pq:</div>
<div>        _, u = heapq.heappop(pq)</div>
<div>        if u == goal:</div>
<div>            break</div>
<div>        for v, weight in graph[u]:</div>
<div>            new_dist = dist[u] + weight</div>
<div>            if new_dist &lt; dist[v]:</div>
<div>                dist[v] = new_dist</div>
<div>                priority = new_dist + heuristic[v]</div>
<div>                heapq.heappush(pq, (priority, v))</div>
<div>                came_from[v] = u</div>
<div>    if dist[goal] == float('inf'):</div>
<div>        print("No path found.")</div>
<div>    else:</div>
<div>        print("A* path cost from vertex", start, "to vertex", goal, "is:", dist[goal])</div>
<div>        # Reconstruct and print path</div>
<div>        path = []</div>
<div>        step = goal</div>
<div>        while step != -1:</div>
<div>            path.append(step)</div>
<div>            step = came_from[step]</div>
<div>        path.reverse()</div>
<div>        print("Path:", " -> ".join(map(str, path)))</div>
<div>def main():</div>
<div>    n = int(input("Enter number of vertices: "))</div>
<div>    m = int(input("Enter number of edges: "))</div>
<div>    graph = [[] for _ in range(n)]</div>
<div>    heuristic = [0] * n</div>
<div>    print("Enter edges (u v w) where u and v are 0-based vertex indices and w is the weight:")</div>
<div>    for _ in range(m):</div>
<div>        u, v, w = map(int, input().split())</div>
<div>        graph[u].append((v, w))</div>
<div>        # graph[v].append((u, w)) # Uncomment if the graph is undirected</div>
<div>    print("Enter heuristic values for each vertex:")</div>
<div>    for i in range(n):</div>
<div>        heuristic[i] = int(input())</div>
<div>    start = int(input("Enter starting vertex: "))</div>
<div>    goal = int(input("Enter goal vertex: "))</div>
<div>    a_star(start, goal, graph, heuristic, n)</div>
<div>if __name__ == "__main__":</div>
<div>    main()</div>
`,

    }

    $("#c").on("click",function(){
        lang = "c";
        $("#c").addClass("shadow");
        $("#cpp").removeClass("shadow");
        $("#java").removeClass("shadow");
        $("#py").removeClass("shadow");
        placeCode()
    });

    $("#cpp").on("click",function(){
        lang = "cpp";
        $("#cpp").addClass("shadow");
        $("#c").removeClass("shadow");
        $("#java").removeClass("shadow");
        $("#py").removeClass("shadow");
        placeCode()
    });

    $("#java").on("click",function(){
        lang = "java";
        $("#java").addClass("shadow");
        $("#cpp").removeClass("shadow");
        $("#c").removeClass("shadow");
        $("#py").removeClass("shadow");
        placeCode()
    });

    $("#py").on("click",function(){
        lang = "py";
        $("#py").addClass("shadow");
        $("#cpp").removeClass("shadow");
        $("#java").removeClass("shadow");
        $("#c").removeClass("shadow");
        placeCode()
    });

    $("#bubble").on("click",function(){
        algo = "b";
        $("#bubble").addClass("shadow");
        $("#select").removeClass("shadow");
        $("#quick").removeClass("shadow");
        $("#insert").removeClass("shadow");
        placeCode()
    });

    $("#select").on("click",function(){
        algo = "s";
        $("#select").addClass("shadow");
        $("#bubble").removeClass("shadow");
        $("#quick").removeClass("shadow");
        $("#insert").removeClass("shadow");
        placeCode()
    });

    $("#quick").on("click",function(){
        algo = "q";
        $("#quick").addClass("shadow");
        $("#select").removeClass("shadow");
        $("#bubble").removeClass("shadow");
        $("#insert").removeClass("shadow");
        placeCode()
    });

    $("#insert").on("click",function(){
        algo = "i";
        $("#insert").addClass("shadow");
        $("#select").removeClass("shadow");
        $("#quick").removeClass("shadow");
        $("#bubble").removeClass("shadow");
        placeCode()
    });

    function placeCode(){
        // C Language
        if(lang=="c" && algo=="b"){
            $("#fetch-code").html(cCode["Bubble"])
        }
    
        if(lang=="c" && algo=="s"){
            $("#fetch-code").html(cCode["DFS"])
        }
    
        if(lang=="c" && algo=="i"){
            $("#fetch-code").html(cCode["DIJKSTRA"])
        }
    
        if(lang=="c" && algo=="q"){
            $("#fetch-code").html(cCode["A* ALGO"])
        }

        //C++ Language
        if(lang=="cpp" && algo=="b"){
            $("#fetch-code").html(cppCode["BFS"])
        }
    
        if(lang=="cpp" && algo=="s"){
            $("#fetch-code").html(cppCode["DFS"])
        }
    
        if(lang=="cpp" && algo=="i"){
            $("#fetch-code").html(cppCode["DIJKSTAR"])
        }
    
        if(lang=="cpp" && algo=="q"){
            $("#fetch-code").html(cppCode["A* ALGO"])
        }

        //Java Language
        if(lang=="java" && algo=="b"){
            $("#fetch-code").html(javaCode["BFS"])
        }

        if(lang=="java" && algo=="s"){
            $("#fetch-code").html(javaCode["DFS"])
        }

        if(lang=="java" && algo=="i"){
            $("#fetch-code").html(javaCode["DIJKSTRA"])
        }

        if(lang=="java" && algo=="q"){
            $("#fetch-code").html(javaCode["A* ALGO"])
        }

        //Python Language
        if(lang=="py" && algo=="b"){
            $("#fetch-code").html(pyCode["BFS"])
        }                    
        if(lang=="py" && algo=="s"){
            $("#fetch-code").html(pyCode["DFS"])
        }           
        if(lang=="py" && algo=="i"){
            $("#fetch-code").html(pyCode["DIJKSTRA"])
        }            
        if(lang=="py" && algo=="q"){
            $("#fetch-code").html(pyCode["A* ALGO"])
        }
    }


    placeCode()


});