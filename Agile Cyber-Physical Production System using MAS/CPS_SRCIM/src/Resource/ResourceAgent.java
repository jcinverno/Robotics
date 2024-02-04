package Resource;

import jade.core.Agent;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;
import jade.core.Agent;
import Libraries.IResource;
import Utilities.DFInteraction;
import Utilities.Constants;
import Libraries.ITransport;

import static Utilities.Constants.ONTOLOGY_EXECUTE_SKILL;

import jade.domain.FIPAAgentManagement.FailureException;
import jade.domain.FIPAAgentManagement.RefuseException;
import jade.domain.FIPAAgentManagement.NotUnderstoodException;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.proto.AchieveREResponder;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.proto.ContractNetResponder;

/**
 * @author Ricardo Silva Peres <ricardo.peres@uninova.pt>
 */
public class ResourceAgent extends Agent {

    String id;
    String next_location, current_location;
    IResource myLib;
    String description;
    String[] associatedSkills;
    String location;

    boolean beingUsed;

    @Override
    protected void setup() {
        Object[] args = this.getArguments();
        this.id = (String) args[0];
        this.description = (String) args[1];
        this.beingUsed = false;

        //Load hw lib
        try {
            String className = "Libraries." + args[2];
            Class cls = Class.forName(className);
            Object instance;
            instance = cls.newInstance();
            myLib = (IResource) instance;
        } catch (ClassNotFoundException | InstantiationException | IllegalAccessException ex) {
            Logger.getLogger(ResourceAgent.class.getName()).log(Level.SEVERE, null, ex);
        }

        this.location = (String) args[3];

        myLib.init(this);
        this.associatedSkills = myLib.getSkills();
        System.out.println("Resource Deployed: " + this.id + " Executes: " + Arrays.toString(associatedSkills));

        //TO DO: Register in DF with the corresponding skills as services

        try {
            DFInteraction.RegisterInDF(this, this.associatedSkills, Constants.DFSERVICE_RESOURCE);
            System.out.println("Registered in DF " + this.getLocalName() + "SKILLS " + Arrays.toString(this.associatedSkills));
        } catch (FIPAException ex) {
            Logger.getLogger(ResourceAgent.class.getName()).log(Level.SEVERE, null, ex);
        }

        // TO DO: Add responder behaviour/s
        this.addBehaviour(new responder(this, MessageTemplate.MatchPerformative(ACLMessage.CFP)));
        this.addBehaviour( new FIPAresponderRE(this, MessageTemplate.MatchPerformative(ACLMessage.REQUEST)) );
    }

    @Override
    protected void takeDown() {
        super.takeDown();
    }


    /**************contract net responder***************/
    private class responder extends ContractNetResponder {
        public responder(Agent a, MessageTemplate mt) {
            super(a, mt);
        }

        @Override
        //protected ACLMessage handleRequest (ACLMessage request) throws NotUnderstoodException, RefuseException
        protected ACLMessage handleCfp(ACLMessage cfp) throws RefuseException, FailureException, NotUnderstoodException {
            System.out.println(myAgent.getLocalName() + ": Processing CFP message");
            String skill = cfp.getContent();
            ACLMessage msg = cfp.createReply();
            String proposal = Integer.toString((int) Math.random());
            if (!beingUsed) {
                for (int i = 0; i < associatedSkills.length; i++ ){
                    if (associatedSkills[i].equals(skill)){
                        msg.setPerformative(ACLMessage.PROPOSE);
                        msg.setContent(proposal);
                        System.out.println(this.getAgent().getLocalName() + ": sent CFP PROPOSAL to " + cfp.getSender().getLocalName());
                    }

                }

            } else {
                msg.setPerformative(ACLMessage.REFUSE);
                System.out.println(this.getAgent().getLocalName() + ": sent CFP REFUSE to " + cfp.getSender().getLocalName());
            }

            return msg;
        }

        @Override
        // protected ACLMessage prepareResultNotification (ACLMessage request,ACLMessage response) throws FailureException
        protected ACLMessage handleAcceptProposal(ACLMessage cfp, ACLMessage propose, ACLMessage accept) throws FailureException {

            System.out.println(myAgent.getLocalName() + ": CFP PROPOSAL accepted by: " + cfp.getSender().getLocalName() + " executing: " + id);
            ACLMessage msg = cfp.createReply();
            msg.setPerformative(ACLMessage.INFORM);
            msg.setContent(location);
            beingUsed = true;
            return msg;
        }
    }

    private class FIPAresponderRE extends AchieveREResponder {
        public FIPAresponderRE(Agent a, MessageTemplate mt) {
            super(a, mt);
        }

        @Override
        protected ACLMessage handleRequest(ACLMessage request) throws NotUnderstoodException, RefuseException {

            System.out.println(myAgent.getLocalName() + ": Processing REQUEST for Resource");
            String my_resource = request.getContent();

            ACLMessage msg = request.createReply();
            msg.setPerformative(ACLMessage.AGREE);
            msg.setContent(my_resource);
            //msg.setOntology(ONTOLOGY_EXECUTE_SKILL);
            return msg;
        }

        @Override
        protected ACLMessage prepareResultNotification (ACLMessage request, ACLMessage response) throws FailureException{
            System.out.println(myAgent.getLocalName() +": Executed Skill Requested");
            /**/
            ACLMessage msg = request.createReply();
            msg.setContent(request.getContent());

            beingUsed = true;

            myLib.executeSkill(msg.getContent());
            beingUsed = false;

            msg.setPerformative(ACLMessage.INFORM);
            return msg;
        }
    }

}
