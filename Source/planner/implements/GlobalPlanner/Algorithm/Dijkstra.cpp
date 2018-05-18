#include "Dijkstra.h"
#include <algorithm>

#include <Console/Console.h>

namespace NS_Planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx,
		int ny) :
		Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
	// priority buffers
	buffer1_ = new int[PRIORITYBUFSIZE];
	buffer2_ = new int[PRIORITYBUFSIZE];
	buffer3_ = new int[PRIORITYBUFSIZE];

	priorityIncrement_ = 2 * neutral_cost_;
}

DijkstraExpansion::~DijkstraExpansion() {
	delete[] buffer1_;
	delete[] buffer2_;
	delete[] buffer3_;
	if (pending_)
		delete[] pending_;
}

void DijkstraExpansion::pushCur(int n, unsigned char* costs) {
	printf("dijk n = %d\n", n);
	printf("pending[n] = %d\n", pending_[n]);
	printf("costs[n] = %d",costs[n]);
	printf("get cost = %d\n", getCost(costs, n));
	printf("currentEnd_ = %d\n", currentEnd_);

	if (n >= 0&& n<ns_ && !pending_[n] &&
	getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE) {
		currentBuffer_[currentEnd_++] = n;
		pending_[n] = true;
	}
}
void DijkstraExpansion::pushNext(int n, unsigned char* costs) {
	if (n >= 0&& n<ns_ && !pending_[n] &&
	getCost(costs, n)<lethal_cost_ && nextEnd_<PRIORITYBUFSIZE) {
		nextBuffer_[nextEnd_++] = n;
		pending_[n] = true;
	}
}
void DijkstraExpansion::pushOver(int n, unsigned char* costs) {
	if (n >= 0&& n<ns_ && !pending_[n] &&
	getCost(costs, n)<lethal_cost_ && overEnd_<PRIORITYBUFSIZE) {
		overBuffer_[overEnd_++] = n;
		pending_[n] = true;
	}
}
//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys) {
	Expander::setSize(xs, ys);
	if (pending_)
		delete[] pending_;

	pending_ = new bool[ns_]; // ns_ = nx_ * ny_   protected
	memset(pending_, 0, ns_ * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)

bool DijkstraExpansion::calculatePotentials(unsigned char* costs,
		double start_x, double start_y, double end_x, double end_y, int cycles,
		float* potential) {

	printf("calculatePotentials running...\n");
	printf("unknown_ = %d,factor_ = %.4f,neutral_cost_ = %d\n",unknown_,factor_,neutral_cost_);
	cells_visited_ = 0;
	// priority buffers
	threshold_ = lethal_cost_;
	currentBuffer_ = buffer1_;
	currentEnd_ = 0;
	nextBuffer_ = buffer2_;
	nextEnd_ = 0;
	overBuffer_ = buffer3_;
	overEnd_ = 0;

	memset(pending_, 0, ns_ * sizeof(bool));

	std::fill(potential, potential + ns_, POT_HIGH);

	// set goal
	int k = toIndex(start_x, start_y);

	if (precise_) {
		double dx = start_x - (int) start_x, dy = start_y - (int) start_y;
		dx = floorf(dx * 100 + 0.5) / 100;
		dy = floorf(dy * 100 + 0.5) / 100;
		potential[k] = neutral_cost_ * 2 * dx * dy;
		potential[k + 1] = neutral_cost_ * 2 * (1 - dx) * dy;
		potential[k + nx_] = neutral_cost_ * 2 * dx * (1 - dy);
		potential[k + nx_ + 1] = neutral_cost_ * 2 * (1 - dx) * (1 - dy); //*/

//		pushCur(k + 2, costs);
//		pushCur(k - 1, costs);
//		pushCur(k + nx_ - 1, costs);
//		pushCur(k + nx_ + 2, costs);
//
//		pushCur(k - nx_, costs);
//		pushCur(k - nx_ + 1, costs);
//		pushCur(k + nx_ * 2, costs);
//		pushCur(k + nx_ * 2 + 1, costs);

		pushCur(k+2,costs);
	      push_cur(k - 1);
	      push_cur(k + nx_ - 1);
	      push_cur(k + nx_ + 2);

	      push_cur(k - nx_);
	      push_cur(k - nx_ + 1);
	      push_cur(k + nx_ * 2);
	      push_cur(k + nx_ * 2 + 1);
	} else {
		potential[k] = 0;
	      push_cur(k + 1);
	      push_cur(k - 1);
	      push_cur(k - nx_);
	      push_cur(k + nx_);
	}

	int nwv = 0;            // max priority block size
	int nc = 0;            // number of cells put into priority blocks
	int cycle = 0;        // which cycle we're on

	// set up start cell
	int startCell = toIndex(end_x, end_y);

	printf("Beforing for loop...\n");

	for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
			{
		if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
				{
			printf("priority blocks empty\n");
			return false;
		}

		// stats
		nc += currentEnd_;
		if (currentEnd_ > nwv)
			nwv = currentEnd_;

		// reset pending_ flags on current priority buffer
		int *pb = currentBuffer_;
		int i = currentEnd_;
		while (i-- > 0)
			pending_[*(pb++)] = false;

		// process current priority buffer
		pb = currentBuffer_;
		i = currentEnd_;
		while (i-- > 0)
			updateCell(costs, potential, *pb++);

		// swap priority blocks currentBuffer_ <=> nextBuffer_
		currentEnd_ = nextEnd_;
		nextEnd_ = 0;
		pb = currentBuffer_;        // swap buffers
		currentBuffer_ = nextBuffer_;
		nextBuffer_ = pb;

		// see if we're done with this priority level
		if (currentEnd_ == 0) {
			threshold_ += priorityIncrement_;    // increment priority threshold
			currentEnd_ = overEnd_;    // set current to overflow block
			overEnd_ = 0;
			pb = currentBuffer_;        // swap buffers
			currentBuffer_ = overBuffer_;
			overBuffer_ = pb;
		}

		// check if we've hit the Start cell
		if (potential[startCell] < POT_HIGH)
			break;
	}

	printf("---------After for loop...----------------\n");

	//ROS_INFO("CYCLES %d/%d ", cycle, cycles);
	if (cycle < cycles)
		return true; // finished up here
	else
		return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs,
		float* potential, int n) {
	cells_visited_++;

	// do planar wave update
	float c = getCost(costs, n);
	if (c >= lethal_cost_)    // don't propagate into obstacles
		return;

	float pot = p_calc_->calculatePotential(potential, c, n);

	// now add affected neighbors to priority blocks
	if (pot < potential[n]) {
		float le = INVSQRT2 * (float) getCost(costs, n - 1);
		float re = INVSQRT2 * (float) getCost(costs, n + 1);
		float ue = INVSQRT2 * (float) getCost(costs, n - nx_);
		float de = INVSQRT2 * (float) getCost(costs, n + nx_);
		potential[n] = pot;

		if (pot < threshold_)    // low-cost buffer block
				{
			if (potential[n - 1] > pot + le)
				pushNext(n - 1, costs);
			if (potential[n + 1] > pot + re)
				pushNext(n + 1, costs);
			if (potential[n - nx_] > pot + ue)
				pushNext(n - nx_, costs);
			if (potential[n + nx_] > pot + de)
				pushNext(n + nx_, costs);
		} else            // overflow block
		{
			if (potential[n - 1] > pot + le)
				pushOver(n - 1, costs);
			if (potential[n + 1] > pot + re)
				pushOver(n + 1, costs);
			if (potential[n - nx_] > pot + ue)
				pushOver(n - nx_, costs);
			if (potential[n + nx_] > pot + de)
				pushOver(n + nx_, costs);
		}
	}
}

} //end namespace global_planner
