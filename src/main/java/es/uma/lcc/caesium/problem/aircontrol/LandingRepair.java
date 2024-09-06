package es.uma.lcc.caesium.problem.aircontrol;

import java.util.ArrayList;
import java.util.List;

/**
 * Repairing: given a list of landing slots sorted by landing time, it is traversed and landing times are adjusted
 * to fulfill the constraints. If a flight appears more than once, only the first occurrence is considered.
 * @author ccottap
 * @version 1.0
 *
 */
public class LandingRepair {
	/**
	 * the problem instance
	 */
	private AirControlProblem data;
	/**
	 * a landing decoder to repair
	 */
	private LandingDecoder ld;
	
	/**
	 * Creates the operator. 
	 */
	public LandingRepair() {
		ld = new LandingDecoder();
		setData(null);
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
		ld.setProblemData(data);
	}

	
	/**
	 * Repairs the landing slots in {@code original} and puts the result into {@code destination}.
	 * The resulting list has no repeated flights and all landing times are feasible.
	 * Returns the number or landing slots processed.
	 * @param original original (potentially infeasible) list of landing slots
	 * @param destination repaired list of landing slots.
	 * @return the number of landing slots processed.
	 */
	public int repair(List<LandingInformation> original, List<LandingInformation> destination) {
		List<String> flightOrder = new ArrayList<String> (data.getNumFlights());
		List<Integer> runways =  new ArrayList<Integer> (data.getNumFlights());
		
		original.sort(null);
		for (var li: original) {
			if (!flightOrder.contains(li.flightID())) {
				flightOrder.add(li.flightID());
				runways.add(li.runway());
			}
		}
		List<LandingInformation> decode = ld.decode(flightOrder, runways);
		
		destination.clear();
		destination.addAll(decode);
		
		return data.getNumFlights(); 
	}

	
	

	@Override
	public String toString() {
		return "LandingRepair";
	}

}
