<template>
  <div>
    <div>
      <b-input-group>
        <b-input-group-addon>String</b-input-group-addon>
        <b-form-input v-model="string"
                      @keydown.native="keydown">
                      placeholder="Please enter a string here"></b-form-input>
      </b-input-group>
    </div>
    <div v-if="historySize > 0">
      <b-list-group>
        <b-list-group-item v-for="(item, idx) in stringHistory"
                           v-if="idx < historySize"
                           v-text="item"
                          :key="idx"
                          href="#"
                          @click="sendGoal(item)">
        </b-list-group-item>
      </b-list-group>
    </div>
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
    },
    historySize: {
      default: 5,
      type: Number
    }
  },
  data () {
    return {
      string: '',
      stringHistory: [],
      actionClient: null,
      actionGoal: null
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
        actionName: 'hmi_msgs/CastAction'
      })
    },
    keydown (e) {
      if (event.which === 13) { // enter
        this.sendGoal(this.string)
        this.stringHistory.unshift(this.string)
        this.string = ''
        while (this.stringHistory.length > this.historySize) {
          this.stringHistory.pop()
        }
      }
    },
    sendGoal (string) {
      this.actionGoal = new ROSLIB.Goal({
        actionClient: this.actionClient,
        goalMessage: {
          message: string
        }
      })
      this.actionGoal.send()
      console.log('Sending goal', this.actionGoal)
    }
  }
}
</script>
