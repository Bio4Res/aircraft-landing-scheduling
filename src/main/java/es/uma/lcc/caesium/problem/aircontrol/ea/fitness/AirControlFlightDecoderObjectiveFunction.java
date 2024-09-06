package es.uma.lcc.caesium.problem.aircontrol.ea.fitness;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.OptimizationSense;
import es.uma.lcc.caesium.ea.fitness.PermutationalObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingDecoder;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;

/**
 * Objective function for the Air Control Problem. Solutions are represented as 
 * permutations of the flights, and a decoder is used to determine the precise landing 
 * information.
 * @author ccottap
 * @version 1.0
 */
public class AirControlFlightDecoderObjectiveFunction extends PermutationalObjectiveFunction implements AirControlObjectiveFunction {
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
	public AirControlFlightDecoderObjectiveFunction(AirControlProblem acp) {
		super(acp.getNumFlights());
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
		List<String> order = new ArrayList<String>(l);
		for (int k=0; k<l; k++) {
			order.add(acp.getFlightID((int)g.getGene(k)));
		} 
		return ld.decode(order);
	}

	@Override
	protected double _evaluate(Individual i) {
		var wait = acp.waitingTime(decode(i.getGenome()));
		long total = 0;
		for (var e: wait.entrySet()) { 
			long w = e.getValue();
			total += w*w;
		}
		addExtraCost(acp.getNumRunways()-1);
		return total;
	}
	
	@Override
	public AirControlProblem getProblemData() {
		return acp;
	}

}
