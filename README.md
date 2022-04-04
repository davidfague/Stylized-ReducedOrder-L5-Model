# Stylized Reduced-Order L5 Model
 
Projects:

(1)	Synaptic input:

(1.1) deliver a random Poisson distribution of stim to each synapse
	Note: 
		getting an error appending stim object list

		Made the following changes:
			use netstim() documentation from neuron

			create a list of stimulus obj,  assign each obj to one synapse
			set_netstim(self) loop on self.stim=[]?
			under "setup synaptic input event append stim
			initialize empty list for self.stim=stim		
			give index for cell.add_synapse(self.stim[i])	

			stim.noise=1 completely random event occurence, stim.noise=0 regular intervals

(1.2) randomly distribute synapses on dendrites according to normal distribution
synapse densities for exc(2.16/um)*200um= 432 and inhib(.22/um)*200um= 44 
	Note: 
		Made the following changes:
			location:
			random number uniform distribution from 0-1
			**(choose variable name)=numpy.random.rand(# of synapses) #choose a variable (this works because cell.add_synapse location parameter is a proportion: 0-1)
		
			Quantity: implement use of the synapse density value instead of raw quanitity


			assigning exc/inhib:
			can use proportion, prop=432/476
			Syn_type = ['exc' if y<prop else
				'inh' for y in **] # for each y in the loop check this condition


(1.3) random seeding to keep the random set of values
	Note:
		Made the following changes:
			numpy.random.seed
			check the documentation
		

(2) 	Future Considerations:
	(2.0) Conductances: #would need to be fit to reduced order
	Ben used a log-normal distribution with mean 0.2 nS and std of 0.345 nS for excitatory synaptic conductances. 
	We fixed inhibitory synaptic conductances at 1 nS 
	(2.1) adjust length of apical and basal dendrites to meet same prop as Ben's L5 (apical/basal = 7440/4640)
	(2.2) add synapses to soma
	(2.3) add axon





