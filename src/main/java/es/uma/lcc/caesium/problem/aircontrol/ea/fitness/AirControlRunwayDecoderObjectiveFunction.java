package es.uma.lcc.caesium.problem.aircontrol.ea.fitness;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.DiscreteObjectiveFunction;
import es.uma.lcc.caesium.ea.fitness.OptimizationSense;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingDecoder;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Objective function for the Air Control Problem. Solutions are represented as a
 * list of runways, of the same length as the number of flights. This is decoded by 
 * sorting the flights on each runway by their arrival time, and assigning each 
 * flight the earliest possible landing time on their assigned runway.
 * @author ccottap
 * @version 1.0
 */
public class AirControlRunwayDecoderObjectiveFunction extends DiscreteObjectiveFunction implements AirControlObjectiveFunction {
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	/**
	 * decoder
	 */
	private LandingDecoder ld;
	
	/**
	 * Basic constructor of the objective function
	 * @param acp the problem instance
	 */
	public AirControlRunwayDecoderObjectiveFunction(AirControlProblem acp) {
		super(acp.getNumFlights(), acp.getNumRunways());
		this.acp = acp;
		ld = new LandingDecoder(acp);
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
		Map<String, Integer> runways = new HashMap<String, Integer>(l);
		for (int k=0; k<l; k++) {
			runways.put(acp.getFlightID(k), (int)g.getGene(k));
		} 
		return ld.decode(runways);
	}
	
	
	/**
	 * Returns the genotype that encodes a landing information. Due to the mechanics
	 * of the decoding mechanism, it cannot be ensured that the information can be
	 * perfectly encoded (it will, if the flights assigned to each runway haven landing 
	 * times whose relative orders matches that of their arrival times at said runway). 
	 * @param info the list of landing slots
	 * @return a genotype encoding this information
	 */
	public Genotype encode (List<LandingInformation> info) {
		Genotype g = new Genotype(acp.getNumFlights());
		for (LandingInformation li: info) {
			g.setGene(acp.getFlightIndex(li.flightID()), li.runway());
		}
		return g;
	}

	@Override
	protected double _evaluate(Individual i) {
		var wait = acp.waitingTime(decode(i.getGenome()));
		long total = 0;
		for (var e: wait.entrySet()) { 
			long w = e.getValue();
			total += w*w;
		}
		return total;
	}

	@Override
	public AirControlProblem getProblemData() {
		return acp;
	}

}
