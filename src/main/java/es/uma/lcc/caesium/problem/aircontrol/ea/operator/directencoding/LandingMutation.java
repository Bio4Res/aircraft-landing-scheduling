package es.uma.lcc.caesium.problem.aircontrol.ea.operator.directencoding;

import java.util.ArrayList;
import java.util.List;

import es.uma.lcc.caesium.ea.base.Genotype;
import es.uma.lcc.caesium.ea.base.Individual;
import es.uma.lcc.caesium.ea.fitness.ObjectiveFunction;
import es.uma.lcc.caesium.ea.operator.variation.mutation.MutationOperator;
import es.uma.lcc.caesium.ea.util.EAUtil;
import es.uma.lcc.caesium.problem.aircontrol.AirControlProblem;
import es.uma.lcc.caesium.problem.aircontrol.LandingInformation;
import es.uma.lcc.caesium.problem.aircontrol.LandingRepair;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlObjectiveFunction;
import es.uma.lcc.caesium.problem.aircontrol.ea.fitness.AirControlPenaltyObjectiveFunction;

/**
 * Local search mutation mutation: an improved solution is sought by changing landing runways for flights.
 * @author ccottap
 * @version 1.0
 *
 */
public class LandingMutation extends MutationOperator {
	/**
	 * repairer
	 */
	private LandingRepair lr;
	/**
	 * the problem instance
	 */
	private AirControlProblem acp;
	
	/**
	 * Creates the operator. 
	 * @param pars String representation of the mutation probability and the number of neighbors to consider
	 */
	public LandingMutation(List<String> pars) {
		super(pars);
		acp = null;
		lr = new LandingRepair(); 
		lr.setData(acp);
	}
	
	@Override
	public void setObjectiveFunction(ObjectiveFunction f) {
		super.setObjectiveFunction(f);
		acp = ((AirControlObjectiveFunction)f).getProblemData();
		lr.setData(acp);
	}

	@Override
	protected Individual _apply(List<Individual> parents) {
		AirControlPenaltyObjectiveFunction p = (AirControlPenaltyObjectiveFunction)obj;
		int numFlights = acp.getNumFlights();
		int numRunways = acp.getNumRunways();
		List<LandingInformation> info = p.decode(parents.get(0).getGenome());
		
		int pos = EAUtil.random(numFlights);
		LandingInformation li = info.get(pos);
		int j = (li.runway() + 1 + EAUtil.random(numRunways-1)) % numRunways;
		
		assert j != li.runway();
		
		info.remove(pos);
		info.add(0, new LandingInformation(li.flightID(), acp.getFlight(li.flightID()).getArrivalTime(j), j));
		
		List<LandingInformation> newInfo = new ArrayList<LandingInformation>(numFlights);
		int cost = lr.repair(info, newInfo);
		obj.addExtraCost((double)cost/(double)numFlights);

		Genotype g = p.encode(newInfo);
		
		Individual ind = new Individual();
		ind.setGenome(g);
		ind.touch();
		return ind;
	}
	

	@Override
	public String toString() {
		return "LandingMutation(" + prob + ")";
	}

}
