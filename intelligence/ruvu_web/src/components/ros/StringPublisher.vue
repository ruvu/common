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
                          @click="publishString(item)">
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
    topicName: {
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
      topic: null
    }
  },
  created () {
    this.readvertise()
  },
  watch: {
    topicName () {
      this.readvertise()
    }
  },
  methods: {
    readvertise () {
      if (this.topic !== null) {
        this.topic.unadvertise()
      }
      console.log(`Advertising to ${this.topicName}`)
      this.topic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.topicName,
        messageType: 'std_msgs/String'
      })
    },
    keydown (e) {
      if (event.which === 13) { // enter
        this.publishString(this.string)
        this.stringHistory.unshift(this.string)
        this.string = ''
        while (this.stringHistory.length > this.historySize) {
          this.stringHistory.pop()
        }
      }
    },
    publishString (string) {
      var msg = new ROSLIB.Message({
        data: string
      })
      this.topic.publish(msg)
    }
  }
}
</script>
