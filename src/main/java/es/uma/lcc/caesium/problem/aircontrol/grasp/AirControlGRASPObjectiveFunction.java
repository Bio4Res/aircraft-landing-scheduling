package es.uma.lcc.caesium.problem.aircontrol.grasp;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import es.uma.lcc.caesium.grasp.base.GRASPObjectiveFunction;
import es.uma.lcc.caesium.grasp.base.LocalSearchResult;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.AirportInformation;
import es.uma.lcc.caesium.problem.aircontrol.Flight;
import es.uma.lcc.caesium.problem.aircontrol.LandingImprovement;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Problem specific functions to solve the Aircraft Landing Scheduling Problem with GRASP
 * @author ccottap
 * @version 1.1
 */
public class AirControlGRASPObjectiveFunction implements GRASPObjectiveFunction {
	/**
	 * an air control problem instance
	 */
	private AirControlProblem data;
	/**
	 * for local improvement
	 */
	private LandingImprovement ls;
	/**
	 * default value of the number of neighbors to explore during local search
	 */
	private final static int NUM_NEIGHBORS = 0;
	/**
	 * to control verbosity
	 */
	private int verbosityLevel = 0;

	/**
	 * default constructor
	 */
	public AirControlGRASPObjectiveFunction() {
		data = null;
		ls = new LandingImprovement(NUM_NEIGHBORS);
	}
	
	/**
	 * Creates the objective function given a problem instance
	 * @param data the problem instance
	 */
	public AirControlGRASPObjectiveFunction(AirControlProblem data) {
		this();
		setProblemData(data);
	}
	
	/**
	 * Sets the verbosity level (0 = no verbosity)
	 * @param verbosityLevel the verbosity level 
	 */
	public void setVerbosityLevel(int verbosityLevel) {
		this.verbosityLevel = verbosityLevel;
	}

	/**
	 * Sets the problem data
	 * @param data an air control problem instance
	 */
	public void setProblemData (AirControlProblem data) {
		this.data = data;
		ls.setData(data);
	}
	
	/**
	 * Sets the number of neighbors to explore during local search
	 * @param num number of neighbors to explore during local search
	 */
	public void setNumNeighbors (int num) {
		ls.setNumNeighbors(num);
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public int getNumberOfVariables() {
		return data.getNumFlights();
	}
	
	@Override
	public double equivalentCost() {
		// There are (n-i+1)*m candidates in the i-th stage (i from 1 to n).
		// The total number of candidates checked is therefore (n+1)*n*m/2.
		// A solution involves n landings. Therefore, a single iteration of the 
		// construction phase is equivalent to (n+1)*n*m/(2*n) = (n+1)*m/2 evaluations
		return (double)(data.getNumFlights()+1)*data.getNumRunways()/2.0;
	}

	
	/**
	 * Returns a list of landing information records. Each of them corresponds to one
	 * of the remaining flights landing at the earliest possible time, given the previous landings.
	 * @param remaining the ids of the remaining flights
	 * @param ai airport information with the time and type of the last landing in each runway
	 * @return a list of landing information records
	 */
	public List<LandingInformation> candidates (Collection<String> remaining, AirportInformation ai) {
		List<LandingInformation> info = new ArrayList<LandingInformation>(remaining.size());
		int numRunways = data.getNumRunways();
		for (String id: remaining) {
			Flight f = data.getFlight(id);
			for (int j=0; j<numRunways; j++) {
				long expected = f.getArrivalTime(j);

				if (verbosityLevel > 1) {
					System.out.println("runway #" + j + ". Expected time = " + expected);
					if (ai.getTime(j) < 0) {
						System.out.println("Runway available");
					}
					else {
						System.out.println("Last use of runway at t=" + ai.getTime(j) + " by " + ai.getType(j) + " plane");
						System.out.println("it can be used again at t=" + (ai.getTime(j) + data.getSeparation(ai.getType(j), f.getType())));
					}
				}
				long t;
				if (ai.getTime(j) < 0) {
					t = expected;
				}
				else {
					t = Math.max(expected, ai.getTime(j) + data.getSeparation(ai.getType(j), f.getType()));
				}
				info.add(new LandingInformation(id, t, j));
			}
		}
		return info;
	}
	
	
	/**
	 * Recursive method to complete the solution given the current state, and the ranks for the remaining decisions.
	 * @param current the flights that have landed so far, with their landing information
	 * @param remaining remaining flights
	 * @param ai airport information with the time and type of the last landing in each runway
	 * @param ranks remaining decisions
	 * @return the landing information
	 */
	public List<LandingInformation> decode (List<LandingInformation> current, 
											Collection<String> remaining, AirportInformation ai,
											List<Integer> ranks) {
		if (remaining.size() > 0) {
			if (verbosityLevel > 0) {
				System.out.println("Selection #" + (data.getNumFlights() - remaining.size() + 1) + "\n-------------------");
				System.out.println("Remaining: " + remaining);
			}
			List<LandingInformation> cand = candidates (remaining, ai);
			cand.sort(Comparator.comparing(LandingInformation::time));
			LandingInformation selected = cand.get(Math.min(cand.size()-1, ranks.get(0)));
			remaining.remove(selected.flightID());
			current.add(selected);
			ai.land(data.getFlight(selected.flightID()).getType(), selected.runway(), selected.time());
			if (verbosityLevel > 0) {
				System.out.println("Candidates: " + cand);
				System.out.println("Picked the #" + (ranks.get(0)+1) + " posibility: " + selected);
			}
			return decode(current, remaining, ai, ranks.subList(1, ranks.size()));
		}
		else
			return current;
	}
	
	/**
	 * Creates a list of landing assignment given a sequence of ranks for the decisions at each stage.
	 * Ranks 0 means the best possible decision, rank 1 the second-best, and so on. If a certain rank
	 * exceeds the number of possibilities, the last one is picked.
	 * @param ranks a list of ranks for each decision
	 * @return the landing information
	 */
	public List<LandingInformation> decode (List<Integer> ranks) {
		assert (ranks.size() == data.getNumFlights());
		if (verbosityLevel > 0) {
			System.out.println("Ranks: " + ranks);
		}
		List<LandingInformation> info = new ArrayList<LandingInformation>(data.getNumFlights());
		int numRunways = data.getNumRunways();
		
		AirportInformation ai = new AirportInformation(numRunways);

		Set<String> remaining = new HashSet<String>(data.getFlightIDs()); // flight ID of each flight
		
		return decode(info, remaining, ai, ranks);	
	}
	

	@Override
	public LocalSearchResult improve(Object sol) {
		if (ls.getNumNeighbors() > 0) {
			@SuppressWarnings("unchecked")
			List<LandingInformation> info = (List<LandingInformation>) sol;
			List<LandingInformation> newInfo = new ArrayList<LandingInformation>(data.getNumFlights());
			int cost = ls.localSearch(info, newInfo);
			return new LocalSearchResult(newInfo, (double)cost/(double)data.getNumFlights() - 1.0);
		}

		return new LocalSearchResult(sol, 0);
	}

	
	/**
	 * {@inheritDoc}
	 */
	@SuppressWarnings("unchecked")
	@Override
	public double evaluate(Object sol) {
		var wait = data.waitingTime((List<LandingInformation>)sol);
		long total = 0;
		for (var e: wait.entrySet()) { 
			long w = e.getValue();
			total += w*w;
		}
		return total;
	}


}
