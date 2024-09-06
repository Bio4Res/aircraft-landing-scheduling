package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.initialization.InitializationOperator;
import es.uma.lcc.caesium.ea.util.EAUtil;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingDecoder;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;

/**
 * Creates a random feasible solution for the Aircraft Landing Scheduling Problem
 * @author ccottap
 * @version 1.0
 */
public class RandomFeasibleLandingInformation extends InitializationOperator {
	/**
	 * decoder
	 */
	private LandingDecoder ld;
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;	
	/**
	 * Generates the operator
	 * @param pars parameters (none)
	 */
	public RandomFeasibleLandingInformation(List<String> pars) {
		super(pars);
		ld = new LandingDecoder();
	}
	
	@Override
	public void setObjectiveFunction (ObjectiveFunction obj) {
		super.setObjectiveFunction(obj);
		acp = ((AirControlObjectiveFunction)obj).getProblemData();
		ld.setProblemData(acp);
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		AirControlPenaltyObjectiveFunction p = (AirControlPenaltyObjectiveFunction)obj;
		int l = acp.getNumFlights();
		int numRunways = acp.getNumRunways();
		
		List<Integer> seq = EAUtil.randomPermutation(l); 
		List<String> order = new ArrayList<String>(l);
		for (int i=0; i<l; i++)
			order.add(acp.getFlightID(seq.get(i)));
		List<Integer> runways = new ArrayList<Integer>(l);
		for (int i=0; i<l; i++)
			runways.add(EAUtil.random(numRunways));
		List<LandingInformation> info = ld.decode(order, runways);
	
		
		Genotype g = p.encode(info);
		Individual ind = new Individual();
		ind.setGenome(g);
		return ind;
	}

	@Override
	public String toString() {
		return "RandomFeasibleLandingInformation";
	}

}
