<template>
  <teleop-canvas @publish='publishTwistMessage($event)':rate='10'></teleop-canvas>
</template>

<script>
import TeleopCanvas from '@/components/ros/TeleopCanvas'
import ROSLIB from 'roslib'

export default {
  components: {
    TeleopCanvas
  },
  props: {
    ros: {
      required: true,
      type: Object
    },
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
      default: 0.8
    }
  },
  watch: {
    topicName () {
      this.readvertise()
    }
  },
  data () {
    return {
      topic: null
    }
  },
  created () {
    this.readvertise()
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
        messageType: 'geometry_msgs/Twist'
      })
    },
    absClip (value, absValue) {
      return Math.max(Math.min(value, absValue), -absValue)
    },
    publishTwistMessage (e) {
      console.log(e)
      var msg = new ROSLIB.Message({
        linear: {
          x: this.absClip(this.translationalVelocityScaling * e.py,
                          this.translationalVelocityScaling),
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: this.absClip(-this.rotationalVelocityScaling * e.px,
                          this.rotationalVelocityScaling)
        }
      })
      console.log(msg)
      this.topic.publish(msg)
    }
  }
}
</script>

<style>
.teleop-canvas-container {
  width: 100%;
}
</style>
