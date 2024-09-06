package es.uma.lcc.caesium.problem.aircontrol;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import es.uma.lcc.caesium.ea.util.EAUtil;

/**
 * Local search: given a list of landing slots, an improved list is obtained by performing first-ascent local 
 * search on the neighborhood obtained by changing a flight from its current runway to a different one (and adjusting
 * the landing times of all affected flights: some flights may land earlier in the original runway, and some flights 
 * may be displaced to a later tame in the modified runway.
 * 
 * The search is performed for a certain number of iterations, or until the current neighborhood is wholly explored 
 * without improvement, whatever comes first. 
 * @author ccottap
 * @version 1.0
 *
 */
public class LandingImprovement {
	/**
	 * number of neighbors to consider during local search
	 */
	private int numNeighbors;
	/**
	 * the problem instance
	 */
	private AirControlProblem data;
	
	/**
	 * Creates the operator. 
	 * @param num number of neighbors to consider
	 */
	public LandingImprovement(int num) {
		setNumNeighbors(num);
		setData(null);
	}

	
	/**
	 * Returns the number of neighbors
	 * @return the number of neighbors
	 */
	public int getNumNeighbors() {
		return numNeighbors;
	}


	/**
	 * Sets the number of neighbors
	 * @param numNeighbors the number of neighbors
	 */
	public void setNumNeighbors(int numNeighbors) {
		this.numNeighbors = numNeighbors;
	}


	/**
	 * Returns the problem instance on which local search is done
	 * @return the problem instance on which local search is done
	 */
	public AirControlProblem getData() {
		return data;
	}


	/**
	 * Sets the problem instance on which local search is done
	 * @param data the problem instance on which local search is done
	 */
	public void setData(AirControlProblem data) {
		this.data = data;
	}


	/**
	 * Internal record to measure the cost of a move
	 * @param pos position of the move
	 * @param value value of the move
	 * @param newList new landing information
	 * @author ccottap
	 * @version 1.0
	 */
	private record MoveDelta (int pos, long value, List<LandingInformation> newList) {};
	
	/**
	 * Computes the gain obtaining from removing a flight from a certain runway
	 * @param slot the slot to remove
	 * @param runway list of flights landing in the runway
	 * @return position of the flight in the ordered list of flights landing in the runway and 
	 * gain of time when the flight is removed 
	 */
	public MoveDelta moveGain(LandingInformation slot, List<LandingInformation> runway) {
		int pos = runway.indexOf(slot); 
		assert pos >= 0;
		int r = slot.runway();
		long t = slot.time() - data.getEarliestArrivalTime(slot.flightID());
		long gain = t*t; // the delay of the current plane
		long available;
		AirplaneType last = null;
		if (pos == 0)
			available = -1;
		else {
			LandingInformation li = runway.get(pos-1); 
			available = li.time();
			last = data.getFlight(li.flightID()).getType();
		}
		List<LandingInformation> newRunway = new ArrayList<LandingInformation>(runway.subList(0, pos));
		for (int i=pos+1; i<runway.size(); i++) {
			LandingInformation  li = runway.get(i);
			Flight f = data.getFlight(li.flightID());
			if (available < 0) {
				available = f.getArrivalTime(r);
			}
			else {
				available = Math.max(f.getArrivalTime(r), available + data.getSeparation(last, f.getType()));
			}
			last = f.getType();

			long eat = data.getEarliestArrivalTime(li.flightID());
			long t1 = li.time()-eat;
			long t2 = available-eat;
			long d = (t1*t1)-(t2*t2);
			gain += d;
			newRunway.add(new LandingInformation(li.flightID(), available, r));
		}
		
		return new MoveDelta(pos, gain, newRunway);
	}
	
	
	/**
	 * Returns the cost of inserting a flight in a runway
	 * @param flightID ID of the flight
	 * @param r runway in which the insertion is done
	 * @param runway ordered list of flights landing in said runway
	 * @return position of the flight in the ordered list of flights landing in the runway and 
	 * additional delay when the flight is added 
	 */
	private MoveDelta moveCost(String flightID, int r, List<LandingInformation> runway) {
		Flight f = data.getFlight(flightID);
		long t = f.getArrivalTime(r);
		List<LandingInformation> newRunway = new ArrayList<LandingInformation>(runway.size()+1);
		int pos=0;
		while ((pos<runway.size()) && (runway.get(pos).time() <= t)) {
			newRunway.add(runway.get(pos));
			pos++;
		}
				
		long cost = 0;
		long available;
		AirplaneType last = null;
		if (pos == 0)
			available = -1;
		else {
			LandingInformation li = runway.get(pos-1); 
			available = li.time();
			last = data.getFlight(li.flightID()).getType();
		}
		if (available < 0) {
			available = t;
		}
		else {
			available = Math.max(t, available + data.getSeparation(last, f.getType()));
		}
		last = f.getType();
		long delta = (available - data.getEarliestArrivalTime(flightID));
		cost += delta*delta;
		newRunway.add(new LandingInformation(flightID, available, r));
		
		for (int i=pos; i<runway.size(); i++) {
			LandingInformation  li = runway.get(i);
			Flight of = data.getFlight(li.flightID());
			available = Math.max(of.getArrivalTime(r), available + data.getSeparation(last, of.getType()));
			last = of.getType();
			long eat = data.getEarliestArrivalTime(li.flightID());
			long t1 = li.time()-eat;
			long t2 = available-eat;			
			long d = (t2*t2)-(t1*t1);
			cost += d;
			
			newRunway.add(new LandingInformation(li.flightID(), available, r));
		}
		
		
		return new MoveDelta(pos, cost, newRunway);
	}
	
	/**
	 * Performs local search on the landing information by swapping runways. 
	 * @param origin the original list of landing slots
	 * @param destination a potentially improved list of landing slots
	 * @return the number of landing slots modified
	 */
	public int localSearch (List<LandingInformation> origin, List<LandingInformation> destination) {
		int evalCost = 0;
		int numRunways = data.getNumRunways();
		int numFlights = data.getNumFlights();

		var partition = new ArrayList<List<LandingInformation>>(numRunways);
		for (int i=0; i<numRunways; i++)
			partition.add(new ArrayList<LandingInformation>(numFlights));
		Collections.sort(origin);
		for (var li: origin) {
			partition.get(li.runway()).add(li);
		}
		
		List<Integer> perm = EAUtil.randomPermutation(numFlights);
		int k = 0;
		int num = 0;
		while ((k < numFlights) && (num < numNeighbors)) {
			LandingInformation slot = origin.get(perm.get(k++));
			List<LandingInformation> originalRunway = partition.get(slot.runway());
			MoveDelta gain = moveGain(slot, originalRunway);
			evalCost += originalRunway.size()-1-gain.pos();
			long best = Long.MAX_VALUE;
			MoveDelta bestMove = null;
			int bestRunway = -1;
			for (int i=0; i<numRunways; i++) {
				if (i != slot.runway()) {
					MoveDelta cost = moveCost (slot.flightID(), i, partition.get(i));
					evalCost += partition.get(i).size()+1-cost.pos();
					if (cost.value() < best) {
						best = cost.value();
						bestMove = cost;
						bestRunway = i;
					}
				}
				num++;
			}
						
			if (gain.value() > best) {
				origin.removeAll(partition.get(slot.runway()));
				origin.removeAll(partition.get(bestRunway));
				origin.addAll(gain.newList());
				origin.addAll(bestMove.newList());
				partition.set(slot.runway(), gain.newList());
				partition.set(bestRunway, bestMove.newList());
				k = 0;
				perm = EAUtil.randomPermutation(numFlights);
			}
		}
		
		destination.clear();
		for (var l: partition) {
			destination.addAll(l);
		}

		return evalCost;
		
	}

	

	@Override
	public String toString() {
		return "LocalSearch(" + numNeighbors + ")";
	}

}
