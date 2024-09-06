package es.uma.lcc.caesium.problem.aircontrol.ea.fitness;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.DiscreteObjectiveFunction;
import es.uma.lcc.caesium.ea.fitness.OptimizationSense;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.AirplaneType;
import es.uma.lcc.caesium.problem.aircontrol.Flight;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Objective function for the Air Control Problem. Solutions are represented as 
 * a list of landing times followed by landing runways. Some solution might be infeasible.
 * @author ccottap
 * @version 1.0
 */
public class AirControlPenaltyObjectiveFunction extends DiscreteObjectiveFunction implements AirControlObjectiveFunction {
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	/**
	 * earliest landing time of any flight
	 */
	private long minTime;
	/**
	 * maximum separation between flights
	 */
	private long maxSeparation;
	
	
	/**
	 * Basic constructor of the objective function
	 * @param acp the problem instance
	 */
	public AirControlPenaltyObjectiveFunction(AirControlProblem acp) {
		super(2*acp.getNumFlights());
		this.acp = acp;
		int numRunways = acp.getNumRunways();
		minTime = Long.MAX_VALUE;
		long maxTime = -1;
		for (String id: acp.getFlightIDs()) {
			Flight f = acp.getFlight(id);
			for (int j=0; j<numRunways; j++) {
				long t = f.getArrivalTime(j);
				if (t < minTime)
					minTime = t;
				if (t > maxTime)
					maxTime = t;
			}
		}
		maxSeparation = -1;
		for (AirplaneType t1: AirplaneType.values())
			for (AirplaneType t2: AirplaneType.values()) {
				long t = acp.getSeparation(t2, t1);
				if (t > maxSeparation)
					maxSeparation = t;
			}
		int n = acp.getNumFlights();
		maxTime += n * maxSeparation;
		for (int j=0; j<n; j++) {
			setAlphabetSize(j, (int)(maxTime-minTime+1));
			setAlphabetSize(j+n, numRunways);
		}
	}

	@Override
	public OptimizationSense getOptimizationSense() {
		return OptimizationSense.MINIMIZATION;
	}
	
	/**
	 * Returns the landing information encoded in a genotype
	 * @param g the genotype
	 * @return a list of Landing information records
	 */
	public List<LandingInformation> decode (Genotype g) {
		int l = acp.getNumFlights();
		List<LandingInformation> info = new ArrayList<LandingInformation>(l);
		for (int k=0; k<l; k++) {
			info.add(new LandingInformation(acp.getFlightID(k), minTime + (int)g.getGene(k), (int)g.getGene(k+l)));
		} 
		return info;
	}
	
	
	/**
	 * Returns the genotype encoding a certain landing information
	 * @param info a list of Landing information records
	 * @return a genotype encoding the list
	 */
	public Genotype encode (List<LandingInformation> info) {
		int l = acp.getNumFlights();
		Genotype g = new Genotype(2*l);
		for (int k=0; k<l; k++) {
			var li = info.get(k);
			int i = acp.getFlightIndex(li.flightID());
			g.setGene(i, (int)(li.time()-minTime));
			g.setGene(i + l, li.runway());
		} 
		return g;
	}
	
	
	@Override
	protected double _evaluate(Individual i) {
		var info = decode(i.getGenome());
		long last =0;
		for (var li: info) {
			if (li.time() > last)
				last = li.time();
		}
		
		var wait = acp.waitingTime(info);
		long total = 0;
		long penalty = maxSeparation * acp.getNumFlights();
		for (var e: wait.entrySet()) { 
			long w = e.getValue();
			if (w < 0) {
				long d = last + penalty - acp.getEarliestArrivalTime(e.getKey());
				total += d*d*w*w;
			}
			else
				total += w*w;
		}
		return total;
	}
	
	@Override
	public AirControlProblem getProblemData() {
		return acp;
	}

}
