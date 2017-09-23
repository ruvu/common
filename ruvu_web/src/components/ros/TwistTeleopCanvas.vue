<template>
  <teleop-canvas @publish='publishTwistMessage($event)':rate='10'></teleop-canvas>
</template>

<script>
import TeleopCanvas from '@/components/ros/TeleopCanvas'

import ROSLIB from 'roslib'
import ros from '@/services/ros'

export default {
  components: {
    TeleopCanvas
  },
  props: {
    topicName: {
      type: String,
      required: true
    },
    translationalVelocityScaling: {
      type: Number,
      default: 0.5
    },
    rotationalVelocityScaling: {
      type: Number,
      default: 0.5
    }
  },
  watch: {
    topicName () {
      this.readvertise()
    }
  },
  data () {
    return {
      twistTopic: null
    }
  },
  created () {
    this.readvertise()
  },
  methods: {
    readvertise () {
      if (this.twistTopic !== null) {
        this.twistTopic.unadvertise()
      }
      console.log(`Subscribing to ${this.topicName}`)
      this.twistTopic = new ROSLIB.Topic({
        ros: ros,
        name: this.topicName,
        messageType: 'geometry_msgs/Twist'
      })
    },
    publishTwistMessage (e) {
      var msg = new ROSLIB.Message({
        linear: {
          x: e.py,
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: -e.px
        }
      })
      this.twistTopic.publish(msg)
    }
  }
}
</script>

<style>
.teleop-canvas-container {
  width: 100%;
  height: 600px;
}
</style>
