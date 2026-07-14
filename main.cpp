#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <queue>

using namespace std;

// Используем как бесконечность
static const long long INF = (long long)1e18;

struct Edge
{
    int to; // Куда ведет ребро
    long long w; // Вес ребра
};

void dijkstra(int n, int s,
                   const vector<vector<Edge>>& adj,
                   vector<long long>& dist,
                   vector<int>& parent)
{
    dist.assign(n + 1, INF);
    parent.assign(n + 1, -1);

    // Сверху всегда вершина с минимальным расстоянием d
    // Храним пары (d, v)
    priority_queue<pair<long long,int>,
                   vector<pair<long long,int>>,
                   greater<pair<long long,int>>> pq;

    dist[s] = 0;
    pq.push({0, s}); // Старт

    while (!pq.empty())
    {
        auto [d, v] = pq.top();
        pq.pop();

        // Если в куче лежит (d, v), но dist[v] уже меньше
        if (d != dist[v])
            continue;

        // Релаксация ребер из v
        for (const Edge& e : adj[v])
        {
            // Кандидат на улучшение:
            // путь до v + вес ребра (v -> e.to)
            long long nd = dist[v] + e.w;

            if (nd < dist[e.to])
            {
                dist[e.to] = nd;
                parent[e.to] = v;
                pq.push({dist[e.to], e.to});
            }
        }
    }
}

// Восстановление пути
vector<int> restore_path_dijkstra(int s, int t, const vector<int>& parent)
{
    vector<int> path;

    // Если старт = финишу
    if (s == t)
        return {s};

    // Идем назад от t к s по parent[]
    for (int cur = t; cur != -1; cur = parent[cur])
    {
        path.push_back(cur);
        if (cur == s) break;
    }

    reverse(path.begin(), path.end());

    // Если не пришли в s, значит пути нет
    if (path.empty() || path.front() != s)
        path.clear();

    return path;
}

// Флойд
void floyd(int n,
                   const vector<vector<long long>>& w,
                   vector<vector<long long>>& dist,
                   vector<vector<int>>& next)
{
    // dist стартует как матрица весов
    dist = w;

    // next[i][j] = следующая вершина после i на пути к j
    next.assign(n + 1, vector<int>(n + 1, -1));

    // Инициализация next для восстановления пути
    for (int i = 1; i <= n; ++i)
    {
        for (int j = 1; j <= n; ++j)
        {
            if (i != j && dist[i][j] < INF)
                next[i][j] = j;
        }
    }

    // Пробуем улучшить dist[i][j] через промежуточную вершину k
    for (int k = 1; k <= n; ++k)
    {
        for (int i = 1; i <= n; ++i)
        {
            if (dist[i][k] >= INF) continue; // i->k недостижимо

            for (int j = 1; j <= n; ++j)
            {
                if (dist[k][j] >= INF) continue; // k->j недостижимо

                long long nd = dist[i][k] + dist[k][j];

                if (nd < dist[i][j])
                {
                    dist[i][j] = nd;
                    // Первый шаг из i к j теперь такой же, как первый шаг из i к k
                    next[i][j] = next[i][k];
                }
            }
        }
    }
}

// Восстановление пути
vector<int> restore_path_floyd(int u, int v, const vector<vector<int>>& next)
{
    vector<int> path;
    if (u == v) return {u};

    // Если next[u][v] = -1, пути нет
    if (u < 0 || v < 0 || u >= (int)next.size() || v >= (int)next.size())
        return path;

    if (next[u][v] == -1)
        return path;

    path.push_back(u);
    int cur = u;

    // Идем по next, пока не придем в v
    while (cur != v)
    {
        cur = next[cur][v];
        path.push_back(cur);
    }

    return path;
}

int main()
{
    ifstream fin("input.txt");
    ofstream fout("output.txt");

    int choice; // 1 - Дейкстра, 2 - Флойд
    fin >> choice;

    // Кол-во вершин и ребер
    int n, m;
    fin >> n >> m;

    // Ориентированный или нет
    int isDirected;
    fin >> isDirected;

    // Матрица весов - нужна для Флойда
    vector<vector<long long>> w(n + 1, vector<long long>(n + 1, INF));
    for (int i = 1; i <= n; ++i)
        w[i][i] = 0;

    // Список смежности - для Дейкстры
    vector<vector<Edge>> adj(n + 1);

    bool hasNegativeEdge = false;

    // Читаем ребра
    for (int i = 0; i < m; ++i)
    {
        int a, b;
        long long c;
        fin >> a >> b >> c;

        if (a < 1 || a > n || b < 1 || b > n)
        {
            cerr << "Неверный номер вершины в ребре: " << a << " " << b << "\n";
            return 1;
        }

        if (c < 0)
            hasNegativeEdge = true;

        // Заполняем матрицу
        w[a][b] = min(w[a][b], c);
        if (!isDirected)
            w[b][a] = min(w[b][a], c);

        // Заполняем список
        adj[a].push_back({b, c});
        if (!isDirected)
            adj[b].push_back({a, c});
    }

    if (choice == 1)
    {
        int s, t;
        fin >> s >> t; // старт и финиш

        if (s < 1 || s > n || t < 1 || t > n)
        {
            cerr << "Некорректные вершины s или t.\n";
            return 1;
        }

        // Дейкстра не подходит для отрицательных весов.
        if (hasNegativeEdge)
        {
            fout << "Алгоритм Дейкстры\n\n";
            fout << "Ошибка: в графе есть отрицательное ребро.\n";
            fout << "Алгоритм Дейкстры корректен только для весов >= 0.\n";
            fout << "Используйте Флойда–Уоршелла (choice = 2).\n";
            return 0;
        }

        vector<long long> dist;
        vector<int> parent;

        dijkstra(n, s, adj, dist, parent);

        fout << "Алгоритм Дейкстры\n\n";
        fout << "Кратчайшие расстояния от вершины " << s << ":\n";

        for (int i = 1; i <= n; ++i)
        {
            fout << "dist[" << i << "] = ";
            if (dist[i] >= INF / 2)
                fout << "INF\n";
            else
                fout << dist[i] << "\n";
        }

        // Восстановление пути s->t
        fout << "\nПуть от " << s << " к " << t << ":\n";
        auto path = restore_path_dijkstra(s, t, parent);

        if (path.empty() || dist[t] >= INF / 2)
        {
            fout << "Пути не существует.\n";
        }
        else
        {
            for (int v : path)
                fout << v << " ";
            fout << "\nДлина пути: " << dist[t] << "\n";
        }
    }
    else if (choice == 2)
    {
        int u, v;
        fin >> u >> v; // Какую пару вершин хотим восстановить

        if (u < 1 || u > n || v < 1 || v > n)
        {
            cerr << "Некорректные вершины u или v.\n";
            return 1;
        }

        vector<vector<long long>> dist;
        vector<vector<int>> next;

        floyd(n, w, dist, next);

        fout << "Алгоритм Флойда\n\n";

        // Проверка отрицательных циклов:
        // Если dist[i][i] < 0 - в графе есть отрицательный цикл.
        bool hasNegativeCycle = false;
        for (int i = 1; i <= n; ++i)
        {
            if (dist[i][i] < 0)
            {
                hasNegativeCycle = true;
                break;
            }
        }
        if (hasNegativeCycle)
        {
            fout << "Внимание: обнаружен отрицательный цикл (dist[i][i] < 0).\n";
            fout << "Для некоторых пар вершин кратчайший путь не определен.\n\n";
        }

        // Вывод расстояний между всеми парами вершин
        fout << "Матрица кратчайших расстояний (dist[i][j]):\n";

        fout << "   ";
        for (int j = 1; j <= n; ++j)
            fout << j << (j == n ? "\n" : " ");

        for (int i = 1; i <= n; ++i)
        {
            fout << i << ": ";
            for (int j = 1; j <= n; ++j)
            {
                if (dist[i][j] >= INF / 2)
                    fout << "INF";
                else
                    fout << dist[i][j];

                if (j != n) fout << " ";
            }
            fout << "\n";
        }
        fout << "\n";

        // Восстановление пути u->v
        fout << "Путь от " << u << " к " << v << ":\n";
        auto path = restore_path_floyd(u, v, next);

        if (path.empty() || dist[u][v] >= INF / 2)
        {
            fout << "Пути не существует.\n";
        }
        else
        {
            for (int x : path)
                fout << x << " ";
            fout << "\nДлина пути: " << dist[u][v] << "\n";
        }
    }
    return 0;
}
