#include <iostream>
#include <vector>
#include "Graph.h"

//int main(int argc, char* argv[])
int main() {

	try {
		std::vector<Edge> e;
		e.push_back(Edge{ 4, 5, 0.5 });
		e.push_back(Edge{ 1, 3, 1.89 });
		e.push_back(Edge{ 5, 4, 25 });
		e.push_back(Edge{ 4, 2, 11.5 });
		auto ee = Edge{5, 4, 5.5};
		
		auto g = Graph(e);
		g.AddEdge(ee);
		auto f = Graph(9);
		//g.print_graph();
		//f.print_graph();
		std::cout << "weight at 3, 1 " << g.At(3, 1) << "\n";
		std::cout << "connected at 4, 2 " << g.Connected(4, 2) << "\n";
		std::cout << "connected at 2, 2 " << g(2, 2) << "\n";
		
		std::vector<Edge> hrany;
		hrany.push_back(Edge{ 0, 1, 5 });
		hrany.push_back(Edge{ 0, 2, 2.5 });
		hrany.push_back(Edge{ 2, 1, 2 });
		hrany.push_back(Edge{ 1, 4, 3.5 });
		hrany.push_back(Edge{ 3, 2, 4 });
		hrany.push_back(Edge{ 2, 5, 7 });
		hrany.push_back(Edge{ 4, 2, 7.5 });
		hrany.push_back(Edge{ 5, 3, 1 });
		hrany.push_back(Edge{ 5, 6, 2 });
		hrany.push_back(Edge{ 5, 4, 6 });
		hrany.push_back(Edge{ 4, 6, 4 });
		f.AddEdges(hrany);

		//f.print_graph();
		f.Path(0, 6);
		f.AddEdge(Edge{ 8, 7, 10.5 });
		//f.print_graph();
		auto x = f.SpannigTree();
		//x.print_graph();
		x.AddEdge(Edge{ 8, 7, 4 });
		std::cout << "\nHello world!" << std::endl;

		std::vector<Edge> egg = {
			{0,1,151},
			{0,2,545},
			{1,3,323},
			{2,4,174},
			{3,5,156},
			{4,5,246},
			{4,6,100},
			{5,7,363},
			{5,8,157},
			{5,9,171},
			{5,10,184},
			{6,10,83},
			{8,11,171},
			{9,12,174},
			{5,13,462},
			{12,16,135},
			{16,13,212},
			{10,14,192},
			{10,15,224},
			{13,17,462},
			{14,18,382},
			{15,19,217},
			{15,20,209},
			{20,24,116},
			{19,18,111},
			{17,21,335},
			{17,18,208},
			{18,22,342},
			{18,23,251},
			{22,23,157},
			{23,24,180},
		};
		auto grafik = Graph(egg);
		//grafik.AddEdges(*empt);
		//grafik.AddEdge(*xd);

		//grafik.print_graph();
		grafik.Path(22, 7);
		//grafik.AddEdge(Edge{ 24, 4, -77 });
		//auto lol = grafik.SpannigTree();
		//lol.print_graph();
	}
	catch (const std::exception &e) {
		std::cout << e.what() << "\n";
	}

	return 0;
}
