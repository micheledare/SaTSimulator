package detection;

import main.Parameters;
import main.Position;
import main.State;
import main.UAV;
import patterns.SearchPattern;

import java.util.*;
import java.util.stream.Collectors;

public class Distributed extends SolverManager {
	
	// constructor with the list of search patterns
	Distributed(List<SearchPattern> candidate){
        super(candidate, State.getUAVs());
    }

    // constructor with the list of the search patterns and uavs
	Distributed(List<SearchPattern> candidate, List<UAV> uavs) {
        super(candidate, uavs);
    }
     
    // definition of class Node. Each node is a search pattern and a time window. Neighbours are search patterns that are interfering (backward or forward)
    // action is the time at which the search pattern is to be initiated, and 0 if the search pattern will not be searched. 
	// resource is the UAV to which the node is assigned. "noresource" is the dummy resource of action 0
    class Node {
    	private int id; // progressive number from 1
    	private SearchPattern searchPattern; 
    	private double initialTime;
    	private double finalTime;
    	private double windowWidth;
    	private String resource;
    	private double action;
    	private List<Node> backwardNeighbours = new ArrayList<>();
    	private List<Node> forwardNeighbours = new ArrayList<>();
    	
    	// constructor
    	Node(int id, SearchPattern sp, double initialTime, double finalTime) {
    		this.id = id;
    		this.searchPattern = sp;
    		this.initialTime = initialTime;
    		this.finalTime = finalTime;
    		this.windowWidth = finalTime - initialTime;
    		this.resource = "noresource"; // initialised at dummmy resource "noresource"
    		this.action = 0; 
    	}
    	
    	// toString(), gives id, window, action and resource
    	public String toString() {
    		return ((Integer)id).toString() + " window: " + this.initialTime + "-" + this.finalTime + " action: " + this.action + " resource " + this.resource;
    	}
    	
    	// method for setting the action
    	public void setAction(String resource, double action) {
    		this.resource = resource;
    		this.action = action;
    	}
    	
    	// Method for filling the neighbours list. Accepts a list of nodes and fills edges with the calling node. 
    	public void setNeighbours(List<Node> nodeList) {
    		for (Node n : nodeList) {
    			// adds n to the neighbours of the current node if they are interfering (and avoids self loops)
    			if (this.id != n.id) {
    				if (n.finalTime >= this.initialTime && n.initialTime - timeBetween(this, n) <= this.finalTime) // n is in forward interference with this
    					this.forwardNeighbours.add(n);
    				if (this.finalTime >= n.initialTime && this.initialTime - timeBetween(n, this) <= n.finalTime) // n is in backward interference with this
    					this.backwardNeighbours.add(n);
    			}
    		}
    	}  	
    }
    
    // time needed to execute n1 and fly to n2 (assuming each UAV has the same velocity)
	double timeBetween(Node n1, Node n2) {
		return n1.searchPattern.getDuration() + Math.ceil(n1.searchPattern.getOrigin().distance(n2.searchPattern.getOrigin())/Parameters.uavVelocity);
	}
	
	// penalty Phi_F associated to nodes v and w
	public double penalty(Node v, Node w) {
		if (v.action == 0 || w.action == 0 || !v.resource.equals(w.resource)) {
			return 0;
		}
		
		
		double mu = Math.min(Math.max(v.action + timeBetween(v,w) - w.action, 0), Math.max(w.action + timeBetween(w,v) - v.action, 0));
		double toReturn = mu +
				( Math.min(Math.max(w.action - v.action, 0), Math.max(v.action + timeBetween(v, w) - w.action, 0) - mu)) +
				( Math.min(Math.max(v.action - w.action, 0), Math.max(w.action + timeBetween(w, v) - v.action, 0) - mu));
		
		//return Math.min(Math.max(w.action - v.action, 0), Math.max(v.action + timeBetween(v, w) - w.action, 0));
		return toReturn;
	}
      
	// computes the utility of the given node (sum of the neighbours' penalties and its own's)
	public double utility(Node v, double f, boolean print) {
		
		double penalty = 0;
		for (Node w : v.backwardNeighbours) {
			penalty += penalty(w, v);
			if (print && penalty(w,v) > 0) System.out.println("Penalty with backward node " + w.id + ": " + penalty(w,v));
		}
		for (Node w : v.forwardNeighbours) {
			penalty += penalty(v, w);
			if (print && penalty(v,w) > 0) System.out.println("Penalty with forward node " + w.id + ": " + penalty(v,w));
		}
		
		if (print) {
			System.out.println("Utility of node " + v.id + " under action " + v.resource + ", " + v.action + ":");
			System.out.println("penalty " + penalty + ", K*penalty " + Parameters.K*penalty + ", f " + f + ", utility " + (- Parameters.K*penalty + f));
		}
		
		return - Parameters.K*penalty + f;
	}
    
	// from a graph to an assignment
    private HashMap<UAV, ArrayList<SearchPattern>> convertToAssignment(List<Node> graph) {
    	        
    	HashMap<UAV, ArrayList<SearchPattern>> assignments = new HashMap<>();
    	
        // assigning plan to UAVs
    	for (UAV u : State.getUAVs()) {
            assignments.put(u, (ArrayList<SearchPattern>) graph.stream().filter(n -> n.resource.equals(u.getUavName())).sorted(nodeComparator)
    															.map(n -> n.searchPattern).collect(Collectors.toList()));
    	}
		return assignments;
	}
    
    // from an assignment to a plan
	private ArrayList<SearchPattern> convertToPlanList(HashMap<UAV, ArrayList<SearchPattern>> assignment) {
				
		ArrayList<SearchPattern> toReturn = new ArrayList<SearchPattern>();
		for (UAV u : assignment.keySet()) {
			toReturn.addAll(assignment.get(u));
		}
		
		return toReturn;
	}
    
	// feasibility of an assignment
	private boolean isFeasible(HashMap<UAV, ArrayList<SearchPattern>> assignment) {
		
		for (UAV u : assignment.keySet()) {
			List<SearchPattern> partialplan = assignment.get(u);
			
			if (partialplan.isEmpty())
				return false;
			
	        int startTime = 0;
	        SearchPattern previous = null;

			for (SearchPattern sp : partialplan) {
	            startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
	            if (startTime > sp.getMaxT())
	            	return false;
	            previous = sp;
			}
		}
		
		return true;
	}

	// feasibility of a configuration
	private boolean isFeasible(List<Node> graph) {
		
		for (UAV u : State.getUAVs()) {
			List<SearchPattern> partialplan = graph.stream().filter(n -> n.resource.equals(u.getUavName())).sorted(nodeComparator)
												   .map(n -> n.searchPattern).collect(Collectors.toList());
			
			if (partialplan.isEmpty())
				return false;
			
	        int startTime = 0;
	        SearchPattern previous = null;

			for (SearchPattern sp : partialplan) {
	            startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
	            if (startTime > sp.getMaxT())
	            	return false;
	            previous = sp;
			}
		}
		
		return true;
	}

	// comparator to order nodes (which one begins first)
    public Comparator<Node> nodeComparator = new Comparator<Node>() {
        @Override
        public int compare(Node n1, Node n2) {
            return (int) (n1.action - n2.action);
        }
    };
        
	/*
	 * function createGraph(): takes a list of search patterns. Does:
	 * 1. Creates all nodes, duplicating search patterns if necessary
	 * 2. After their creation, puts them in the list nodesList
	 * 3. Spans the list to fill the attribute "neighbours"
	 * 4. Returns the list
	 */
    List<Node> createGraph(List<SearchPattern> candidates){
    	
    	// greedy initialization - produce the greedy schedule
    	HashMap<UAV,ArrayList<SearchPattern>> sequence = new GreedyAlgorithm(candidates).sequence(); // result of greedy algorithm
		HashMap<SearchPattern, Integer> greedyPlan = new HashMap<>(); // greedy schedule
		HashMap<SearchPattern, UAV> whichUAV = new HashMap<>(); // which UAV does each search pattern
        for (UAV u : sequence.keySet()){ 
	        int startTime = 0;
	        SearchPattern previous = null;
	        for (SearchPattern sp : sequence.get(u)){
	            startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
	            greedyPlan.put(sp, startTime);
	            whichUAV.put(sp, u);
	            previous = sp;
	        }
        }
    	
    	List<Node> nodesList = new ArrayList<>();
    	int id = 1;
    	for (SearchPattern sp : candidates) {
    		double numSplits = Math.ceil((sp.getMaxT()-sp.getMinT())/sp.getDuration());
    		double sizeSplits = Math.floor((sp.getMaxT()-sp.getMinT())/numSplits);
    		double ti = sp.getMinT();
    		double tf = ti + sizeSplits;
    		for (int k = 0; k < numSplits; k++) {
    			
    			Node n = new Node(id, sp, ti, tf);
    			if (Parameters.verbose) System.out.println("Node added. Id " + id + " search pattern " + sp.toString() + " (time window " + ti + "-" + tf + ")");
    			id += 1;
        		
        		// greedy initialization
        		if(greedyPlan.containsKey(sp)){
        			if(greedyPlan.get(sp) >= ti && greedyPlan.get(sp) < tf){
        				n.setAction(whichUAV.get(sp).getUavName(), greedyPlan.get(sp));
        				if (Parameters.verbose) System.out.println("Node " + n.id + " initialized at " + n.resource + ", action " + n.action);
        			}
        		}
        		else {
        			if (Parameters.verbose) System.out.println("Node " + n.id + " initialized at " + n.resource + ", action " + n.action);
        		}
        		
    			nodesList.add(n);
				
    			ti = tf;
    			tf = ti + sizeSplits;
    		}
    	}
    	    	
		// filling neighbours
    	for (Node n : nodesList)
    		n.setNeighbours(nodesList);
		
		System.out.println("Created graph. Search patterns: " + candidates.size() + ". Nodes: " + nodesList.size() + "\n");
		
    	return nodesList;
    }
    
    // function that for each UAV gives its plan as a list of SearchPattern
    // it's the function that samples!
    public HashMap<UAV, ArrayList<SearchPattern>> sequence() {  
    	
    	double timezero = System.currentTimeMillis(); // saving initial time
    	double eta = Parameters.eta; // Noise parameter
    	
    	// Definition and creation of the graph
        List<Node> graph = createGraph(candidateSearchPattern); 
                
        // evaluating initial configuration
    	HashMap<UAV, ArrayList<SearchPattern>> bestAssignment = convertToAssignment(graph); // best configuration - assignment
        ArrayList<SearchPattern> bestPlanList = convertToPlanList(bestAssignment); // best configuration - plan
    	Plan plan = createPlan(bestPlanList, graph.size());
    	double fmax = plan.f;
        System.out.println("Starting at assignment " + bestAssignment.toString() + "\nof value " + fmax);
        boolean feas = isFeasible(bestAssignment);
        
        // initialising variables that change at each iteration
        Node sampledNode;
        double u0, p0;
    	Set<Double> partition = new HashSet<>();
    	double timeBetween;
    	Double[] a, b, C;
    	double integral, Z;
        double previousAction, newAction;
        String previousResource;
        double f;
                
        // register of all plans found and their value
        HashMap<HashMap<UAV, ArrayList<SearchPattern>>, Double> register = new HashMap<>();
        HashMap<UAV, ArrayList<SearchPattern>> assignment;
        if (feas) register.put(bestAssignment, fmax);
        
        // Start iterations
        Random r = new Random(Parameters.seed);
        double elapsed = System.currentTimeMillis() - timezero;
        double timeOut = Parameters.planTimeLimit*1000;
        int iterationsCount = 0;
        int feasibleIterations = 0;
		while (elapsed < timeOut) {
			
        	iterationsCount += 1;
        	
        	boolean verbose3 = (iterationsCount <= Parameters.printIterations);
        	
        	// sample random node
        	sampledNode = graph.get(r.nextInt(graph.size()));
        	previousAction = sampledNode.action; 
        	previousResource = sampledNode.resource;
        	
        	// utility of action = 0
        	if (verbose3) {
        		System.out.println("\nSampled node " + sampledNode.id + " of action " + sampledNode.resource + ", " + sampledNode.action + " and window " + sampledNode.initialTime + " - " + sampledNode.finalTime);
        		
        		System.out.println("Neighbours: ");
        		for (Node n : sampledNode.backwardNeighbours) {
        			System.out.println(n.id + " of action " + n.action + " and timebetween " + timeBetween(n, sampledNode) + " and window " + n.initialTime + " - " + n.finalTime);
        		}
        		for (Node n : sampledNode.forwardNeighbours) {
        			System.out.println(n.id + " of action " + n.action + " and timebetween " + timeBetween(sampledNode, n) + " and window " + n.initialTime + " - " + n.finalTime);
        		}
        		
        	}
        	
        	sampledNode.setAction("noresource", 0);
        	// get value of f
        	assignment = convertToAssignment(graph);
        	if (!isFeasible(assignment)) f = 0;
        	else if (register.containsKey(assignment)) f = register.get(assignment);
        	else {
        		plan = updatePlan(plan, convertToPlanList(assignment));
        		f = plan.f;
        		register.put(assignment, f);
        	}
        	u0 = utility(sampledNode, f, verbose3);
        	p0 = Parameters.delta*Math.exp(eta*u0); // unnormalized probability mass of action 0
        	Z = p0; // total unnormalized probability mass (to be incremented with the other actions)
        	if (verbose3) System.out.println("Evaluated action 0. Utility of 0: " + u0 + ", prob: " + p0);
        	
        	// initialize maps: for each UAV, partition, utilities and integrals
        	HashMap<UAV, Double[]> aa = new HashMap<UAV, Double[]>();
        	HashMap<UAV, Double[]> bb = new HashMap<UAV, Double[]>();
        	HashMap<UAV, Double[]> CC = new HashMap<UAV, Double[]>();
        	HashMap<UAV, Double> probs = new HashMap<UAV, Double>(); // for each UAV, the unnormalized probability mass of its actions
        	for (UAV u : State.getUAVs()) { // loop on resources
        		
	        	partition.clear();
	        	double initialTime = sampledNode.initialTime;
	        	double finalTime = sampledNode.finalTime;
        		
	        	// adding knots to the partition
	        	partition.add(initialTime);
	        	partition.add(finalTime);
	        	
	        	for (Node w : sampledNode.backwardNeighbours) {	        		
	        		if (w.action == 0 || !w.resource.equals(u.getUavName())) continue; // only those that are active and under the same resource
	        		timeBetween = timeBetween(w, sampledNode); // time needed to execute n and fly to sampledNode
	        		// those outside sampleNode's time window will be later filtered out
	        		partition.add(w.action);
	        		//partition.add(w.action + timeBetween/2);
	        		partition.add(w.action + timeBetween);
	        	}
	        	for (Node w : sampledNode.forwardNeighbours) {
	        		if (w.action == 0 || !w.resource.equals(u.getUavName())) continue; // only those under the same resource
	        		timeBetween = timeBetween(sampledNode, w); // time needed to execute sampleNode and fly to n
	        		// those outside sampleNode's time window will be later filtered out
	        		partition.add(w.action - timeBetween);
	        		//partition.add(w.action - timeBetween/2);
	        		partition.add(w.action);
	        	}
	        	
	        	// filtering out knots and building vector a
	        	a = partition.stream().filter(d -> d >= initialTime && d <= finalTime).toArray(Double[]::new);       	
	        	Arrays.sort(a);
	        	aa.put(u, a);
	        	
	        	// defining vector b
	        	b = new Double[a.length];
	        	sampledNode.setAction(u.getUavName(), a[0]);
	        	assignment = convertToAssignment(graph);
	        	if (!isFeasible(assignment)) f = 0;
	        	else if (register.containsKey(assignment)) f = register.get(assignment);
	        	else {
	        		plan = updatePlan(plan, convertToPlanList(assignment));
	        		f = plan.f;
	        		register.put(assignment, f);
	        	}
	        	b[0] = utility(sampledNode, f, verbose3);	        	
	        	if (verbose3) System.out.println("Evaluated action " + a[0] + ": utility " + b[0] + ", prob: " + Math.exp(Parameters.eta*b[0]));
	        	for (int i = 1; i < a.length; i++) {
	        		sampledNode.setAction(u.getUavName(), a[i]);
	        		b[i] = utility(sampledNode, f, verbose3);	        		
	        		if (verbose3) System.out.println("Evaluated action " + a[i] + ": utility " + b[i] + ", prob: " + Math.exp(Parameters.eta*b[i]));
	        	}
	        	bb.put(u, b);
	        	
	        	// sampling new action
	        	C = getC(a, b, eta);
	        	integral = 0;
	        	for (int i = 1; i < C.length; i++) {
	        		integral += C[i];
	        	}
	        	CC.put(u, C);
	        	
	        	double probU = integral/(Parameters.nUAV*sampledNode.windowWidth);
	        	//double probU = integral;
	        	if (verbose3) System.out.println("Integral of " + u.getUavName() + ": " + probU);
	        	Z += probU;
	        	probs.put(u, probU);
        	}
        	        	        	
        	double y = r.nextDouble()*Z;
        	if (y < p0) {
        		sampledNode.setAction("noresource", 0);
        		if (verbose3) System.out.println("y: " + y + " Z: " + Z + ", sampled: " + sampledNode.action + ", " + sampledNode.resource);
        	}
        	else {
        		double cumprob = p0;
        		for (UAV u : probs.keySet()) {
        			if (y <= cumprob + probs.get(u)) {
        				newAction = sample(aa.get(u), bb.get(u), CC.get(u), probs.get(u), eta, r);        				
        				sampledNode.setAction(u.getUavName(), newAction);
        				if (verbose3) System.out.println("y: " + y + " Z: " + Z + ", sampled: " + sampledNode.action + ", " + sampledNode.resource);
                		break;
        			}
        			cumprob += probs.get(u);
        		}
        		
        	}

        	// if action didn't change go to next loop iteration
        	if (sampledNode.action == previousAction && sampledNode.resource.equals(previousResource)) {
        		continue;
        	}
    			
    		if (isFeasible(graph)) {
    			feasibleIterations += 1;
	        	assignment = convertToAssignment(graph);
	        	if (!isFeasible(assignment)) f = 0;
	        	else if (register.containsKey(assignment)) f = register.get(assignment);
	        	else {
	        		plan = updatePlan(plan, convertToPlanList(assignment));
	        		f = plan.f;
	        		register.put(assignment, f);
	        	}
		    	if (f > fmax){
		    		fmax = f;
		    		bestAssignment = assignment;
		    		System.out.println("Plan updated. Iteration: " + iterationsCount + " New plan: " + bestAssignment.toString() + " value: " + fmax);
		    	}
    		}
    		elapsed = System.currentTimeMillis() - timezero;
        }
	
		int feasiblePlans = 0;
		for (HashMap<UAV, ArrayList<SearchPattern>> assment : register.keySet()) {
			if (isFeasible(assment)){
				feasiblePlans += 1;
				if(register.get(assment) > fmax) {
					fmax = register.get(assment);
					bestAssignment = assment;
		    		System.out.println("There was a better plan in the register: " + bestAssignment.toString() + " value: " + fmax);
				}
			}
		}
		System.out.println("Iterations " + iterationsCount + ", feasible " + feasibleIterations + ", feasible plans " + feasiblePlans + ", fmax: " + fmax); 
    	return bestAssignment;
    }

    /*
	private double getValue(HashMap<HashMap<UAV, ArrayList<SearchPattern>>, Double> register, List<Node> graph) {
				
		HashMap<UAV, ArrayList<SearchPattern>> assignment = convertToAssignment(graph);
		if (!isFeasible(assignment)) return 0;
    			
    	if (register.containsKey(assignment)) { 
    		return register.get(assignment);
    	}
    	else { // add to register if new
    		double f = evaluate(convertToPlan(assignment));
    		register.put(assignment, f);
    		return f;
    	}	
	}
	*/
    
    // class to memorize a plan along with its arrays to compute the value f(S)
    public class Plan {
    	private ArrayList<SearchPattern> list;
    	private double[] PSarray, pStararray;
    	HashMap<Integer, HashMap<List<Position>, Double>> pDestarray;
    	double f;
    	
    	public Plan(ArrayList<SearchPattern> list, double[] PSarray, double[] pStararray, HashMap<Integer, HashMap<List<Position>, Double>> pDestarray){
    		this.list = list;
    		this.PSarray = PSarray;
    		this.pStararray = pStararray;
    		this.pDestarray = pDestarray;
    	}
    	
    	public Plan(ArrayList<SearchPattern> list, double[] PSarray, double[] pStararray, HashMap<Integer, HashMap<List<Position>, Double>> pDestarray, double f){
    		this.list = list;
    		this.PSarray = PSarray;
    		this.pStararray = pStararray;
    		this.pDestarray = pDestarray;
    		this.f = f;
    	}
    }

	private Plan updatePlan(Plan plan, ArrayList<SearchPattern> newList) {
		
		Plan newPlan = new Plan(newList, plan.PSarray, plan.pStararray, plan.pDestarray);
				
    	// extracts index of first different search pattern
		int minsize = Math.min(plan.list.size(), newList.size());
		int firstDifferent = minsize; // assumed to be one subset of the other. This also over prev = empty
    	for (int k = 0; k < minsize; k++) {
    		if (plan.list.get(k) != newList.get(k)) {
    			firstDifferent = k;
    			break;
    		}
    	}
    	
    	// initializing
    	double newPS, PS, pStar;
    	double gamma;
    	int ind;
    	double previous;
    	int nDestination = cityNameMapping.size();
        double pi = 1. / nDestination;	

		// continuing
		for (int k = firstDifferent; k < newPlan.list.size(); k++) {
			// PS
			if (k == 0) {
				newPS = 0;						
			}
			else {
				PS = plan.PSarray[k-1];
				pStar = plan.pStararray[k-1];
				newPS = PS + pStar*(1-PS);
				//System.out.println("PS update: k = " + k + ", PS = " + newPS);
			}
			newPlan.PSarray[k] = newPS;
			
			// PSdest
			HashMap<List<Position>, Double> toInsert = new HashMap<>();
			if (k == 0) {
				for (List<Position> d : cityNameMapping.keySet()) {
					toInsert.put(d, pi);  // initialised with the uniform a priori
				}
			}
			else {
				gamma = newPlan.list.get(k-1).getGamma(); // gamma_k is needed, but list indexing starts at 0
				for (List<Position> x : cityNameMapping.keySet()) {
					if (newPlan.list.get(k-1).getCompatibleDestinations().contains(x))
						ind = 1;
					else 
						ind = 0;
					
					previous = newPlan.pDestarray.get(k-1).get(x);
					pStar = newPlan.pStararray[k-1];
					toInsert.put(x, previous*(1-gamma*ind)/(1-pStar));
				}
			}
			newPlan.pDestarray.put(k, toInsert);
																							
			// pS 
			pStar = newPlan.list.get(k).getGamma();
			double pC = 0;
			for (List<Position> y : newPlan.list.get(k).getCompatibleDestinations()) {
				pC += newPlan.pDestarray.get(k).get(y);
			}
			pStar *= pC;
			newPlan.pStararray[k] = pStar;
		}
		
		// final
		PS = newPlan.PSarray[newPlan.list.size()-1];
		pStar = newPlan.pStararray[newPlan.list.size()-1];
		PS = PS + pStar*(1-PS);
		newPlan.f = PS;
		return newPlan;
	}

	// computes the integral of piecewise exponential and continuous function exp(eta*u) with u defined by the knots a_i, b_i
	// returns an array of the integrals on each segment [a_(i-1), a_i]
    private Double[] getC(Double[] a, Double[] b, double eta) {
    	int n = a.length;
    	Double[] C = new Double[n];
    	C[0] = 0.0;
    	double previousExp = Math.exp(eta*b[0]);
    	double newExp;
    	for (int i = 1; i < n; i++) {
    		if (b[i].equals(b[i-1])) { // to avoid division by 0
    			C[i] = (a[i]-a[i-1]) * previousExp;
    		}
    		else {
    			newExp = Math.exp(eta*b[i]);
    			C[i] = (a[i]-a[i-1])/(b[i]-b[i-1]) * ( newExp - previousExp )/eta;
    			previousExp = newExp;
    		}
    	}
    	return C;
    }
    
    // sampling of the new action from density 1/integral*(exp(eta*u)) with u piecewise linear and continuous defined by the knots (a_i, b_i)
    private double sample(Double[] a, Double[] b, Double[] C, double integral, double eta, Random r) {
    	int n = a.length;
    	double y = r.nextDouble();
    	double ybar = integral*y;
    	double Ccum = 0;
    	int i = 0;
    	for (int j = 0; j < n; j++) {
    		if (ybar <= Ccum+C[j]) {
    			i = j;
    			break;
    		}
    		Ccum += C[j];
    	}
    	double x;
    	if (b[i].equals(b[i-1])) {
    		x = a[i-1] + (ybar - Ccum)*Math.exp(-eta*b[i-1]); // to avoid division by 0
    	}
    	else {
    		x = a[i-1] + (a[i]-a[i-1])/(b[i]-b[i-1]) * ( - b[i-1] + Math.log(eta*(b[i]-b[i-1])/(a[i]-a[i-1])*(ybar - Ccum) + Math.exp(eta*b[i-1]))/eta);
    	}
		return x;
    }
    
	// method for evaluating f(S)
 	private Plan createPlan(ArrayList<SearchPattern> planSkeleton, int maxsize) {
 		 		
 		if (planSkeleton.isEmpty())
 			return new Plan(planSkeleton, new double[maxsize], new double[maxsize], new HashMap<Integer, HashMap<List<Position>, Double>>(), 0.0);
 		
 		// initializing
 		double[] PSarray = new double[maxsize];
 		double[] pStararray = new double[maxsize];
 		HashMap<Integer, HashMap<List<Position>, Double>> pDestarray = new HashMap<>();
    	double newPS, PS, pStar;
    	double gamma;
    	int ind;
    	double previous;
    	int nDestination = cityNameMapping.size();
        double pi = 1. / nDestination;	

		// continuing
		for (int k = 0; k < planSkeleton.size(); k++) {
			// PS
			if (k == 0) {
				newPS = 0;						
			}
			else {
				PS = PSarray[k-1];
				pStar = pStararray[k-1];
				newPS = PS + pStar*(1-PS);
				//System.out.println("PS update: k = " + k + ", PS = " + newPS);
			}
			PSarray[k] = newPS;
			
			// PSdest
			HashMap<List<Position>, Double> toInsert = new HashMap<>();
			if (k == 0) {
				for (List<Position> d : cityNameMapping.keySet()) {
					toInsert.put(d, pi);  // initialised with the uniform a priori
				}
			}
			else {
				gamma = planSkeleton.get(k-1).getGamma(); // gamma_k is needed, but list indexing starts at 0
				for (List<Position> x : cityNameMapping.keySet()) {
					if (planSkeleton.get(k-1).getCompatibleDestinations().contains(x))
						ind = 1;
					else 
						ind = 0;
					
					previous = pDestarray.get(k-1).get(x);
					pStar = pStararray[k-1];
					toInsert.put(x, previous*(1-gamma*ind)/(1-pStar));
				}
			}
			pDestarray.put(k, toInsert);
																							
			// pS 
			pStar = planSkeleton.get(k).getGamma();
			double pC = 0;
			for (List<Position> y : planSkeleton.get(k).getCompatibleDestinations()) {
				pC += pDestarray.get(k).get(y);
			}
			pStar *= pC;
			pStararray[k] = pStar;
		}
		
		// final
		PS = PSarray[planSkeleton.size()-1];
		pStar = pStararray[planSkeleton.size()-1];
		PS = PS + pStar*(1-PS);
		return new Plan(planSkeleton, PSarray, pStararray, pDestarray, PS);
 	}
	 
	 
	// method for evaluating f(S)
 	private double evaluate(List<SearchPattern> planSkeleton) {
 		 		
 		if (planSkeleton.isEmpty())
 			return 0;
 		
 		// parameters
 		int planLength = planSkeleton.size();
 		int nDestination = cityNameMapping.size();
 		 		
 		// initialisation
 		// TS 
 		//double TS = 0;
 		// PS
 		double PS = 0;
 		// pDest
 		HashMap<List<Position>, Double> pDest = new HashMap<>();
 		double pi = 1.0/nDestination;
 		for (List<Position> d : cityNameMapping.keySet()) {
 			pDest.put(d, pi); // initialised with the uniform a priori
 		}
 		
 		double ind = 0; // initialising for later
 		// pS
 		double pStar = planSkeleton.get(0).getGamma();
 		double pC = 0;
 		for (List<Position> d : planSkeleton.get(0).getCompatibleDestinations()) {
 			pC += pDest.get(d);
 		}
 		pStar *= pC;

 		// updating
 		double gamma;
 		double newPS;
 		for (int k = 1; k < planLength; k++) { // does (planLength-1) loops
 			// PS and TS
 			newPS = PS + pStar*(1-PS);
 			//TS = TS + (planSkeleton.get(k-1).getMinT() + planSkeleton.get(k-1).getMaxT())* 0.5/maxExpectedTime*(newPS - PS);
 			PS = newPS;
 			// PSdest
 			gamma = planSkeleton.get(k-1).getGamma(); // gamma_k is needed, but list indexing starts at 0
 			for (List<Position> x : cityNameMapping.keySet()) {
 				if (planSkeleton.get(k-1).getCompatibleDestinations().contains(x))
 					ind = 1;
 				else 
 					ind = 0;
 				pDest.replace(x, pDest.get(x)*(1-gamma*ind)/(1-pStar));
 			}
 			// PSstar 
 			pC = 0;
 			for (List<Position> y : planSkeleton.get(k).getCompatibleDestinations()) {
 				pC += pDest.get(y);
 			}
 			gamma = planSkeleton.get(k).getGamma();
 			pStar = gamma*pC;
 		}
 		
 		// final
 		newPS = PS + pStar*(1-PS);
 		//TS = TS + (planSkeleton.get(planLength-1).getMinT() + planSkeleton.get(planLength-1).getMaxT())*0.5/maxExpectedTime*(newPS-PS);
 		PS = newPS;
 		//double GS = PS - Parameters.weight*TS;
 		 		
 		return PS;
 	}
    
 	// time needed to execute sp1 and fly to sp2
	double distanceSP(SearchPattern sp1, SearchPattern sp2, UAV u){

        if (sp1 == null){
            return  Math.ceil(u.getCurrent().distance(sp2.getOrigin())/Parameters.uavVelocity);
        }
        return Math.ceil(sp1.getOrigin().distance(sp2.getOrigin())/ Parameters.uavVelocity)+ sp1.getDuration();
    }
    
	// function outputting the result
    public HashMap<String, HashMap<Integer, List<SearchPattern>>> getPlan(){
        HashMap<String,HashMap<Integer,List<SearchPattern>>> timedPlan = new  HashMap<>();
        HashMap<UAV,ArrayList<SearchPattern>> sequence = sequence();
        double fmax = evaluate(convertToPlanList(sequence));
		System.out.println("Solver: " + Parameters.solver + "\nSeed: " + Parameters.seed + "\n" + "fmax value: " + fmax + "\n" + "plan: " + convertToPlanList(sequence).toString());
        for (UAV u : sequence.keySet()){
        	HashMap<Integer,List<SearchPattern>> plan = new HashMap<>();
            int time = State.getTime() + 1;
            int startTime = 0;
            SearchPattern previous = null;
            for (SearchPattern sp : sequence.get(u)){
                List<SearchPattern> tmpList = new ArrayList<>();
                tmpList.add(sp);
                plan.put(time,tmpList);
                startTime =  new Double(Math.max(startTime+distanceSP(previous,sp,u),sp.getMinT())).intValue();
                time = startTime + State.getTime() + new Double(sp.getDuration()).intValue();
                previous = sp;
            }
            timedPlan.put(u.getUavName(),plan);
        }
        return timedPlan;
    }
}
