<template>
  <div>
    <b-input-group>
      <b-input-group-addon>Description</b-input-group-addon>
      <b-form-input placeholder="Description" v-model="description"></b-form-input>
    </b-input-group>
    <b-input-group>
      <b-input-group-addon>Example sentences</b-input-group-addon>
      <b-form-textarea id="textarea1"
                     v-model="exampleSentences"
                     placeholder="sentence1; sentence2"
                     :rows="3"
                     :max-rows="6"></b-form-textarea>
    </b-input-group>
    <b-form-textarea id="grammar" placeholder="Grammar" :rows="5" v-model="grammar"></b-form-textarea>
    <b-input-group>
      <b-input-group-addon>Grammar target</b-input-group-addon>
      <b-form-input placeholder="Grammar target" v-model="target"></b-form-input>
      <b-input-group-addon><b-button @click="sendGoal(grammar, target)">Query</b-button></b-input-group-addon>
    </b-input-group>
    <b-form-textarea id="grammar" placeholder="Query Response" :disabled="true" :rows="5" v-text="result"></b-form-textarea>
  </div>
</template>

<script>
import ROSLIB from 'roslib'

export default {
  props: {
    ros: {
      required: true,
      type: Object
    },
    actionName: {
      required: true,
      type: String
    }
  },
  data () {
    return {
      description: '',
      exampleSentences: '',
      grammar: '',
      target: '',
      result: null
    }
  },
  created () {
    this.readvertise()
  },
  watch: {
    actionName () {
      this.readvertise()
    }
  },
  methods: {
    readvertise () {
      // if (this.topic !== null) {
      //   this.topic.unadvertise()
      // }
      // console.log(`Advertising to ${this.topicName}`)
      this.actionClient = new ROSLIB.ActionClient({
        ros: this.ros,
        serverName: this.actionName,
        actionName: 'hmi_msgs/QueryAction'
      })
    },
    sendGoal (grammar, target) {
      this.actionGoal = new ROSLIB.Goal({
        actionClient: this.actionClient,
        goalMessage: {
          description: this.description,
          example_sentences: this.exampleSentences.split(/;|\n/),
          grammar: this.grammar,
          target: this.target
        }
      })
      this.actionGoal.on('result', (result) => {
        this.result = result
      })
      this.actionGoal.send()
      console.log('Sending goal', this.actionGoal)
    }
  }
}
</script>
