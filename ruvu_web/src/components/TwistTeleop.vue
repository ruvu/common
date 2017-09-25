<template>
  <div class="card">
    <div class="card-header">
      Twist teleop
    </div>
    <div class="card-block">
      <topic-selector :initialTopicName="topicName" @updated="topicName = $event"></topic-selector>
    </div>
    <div class="card-block twist-teleop-canvas-container">
      <twist-teleop-canvas :topicName="topicName"></twist-teleop-canvas>
    </div>
    <div class="card-footer text-muted">
      Publishing to {{topicName}}
    </div>
  </div>
</template>

<script>
import TwistTeleopCanvas from '@/components/ros/TwistTeleopCanvas'
import TopicSelector from '@/components/ros/TopicSelector'

export default {
  components: {
    TwistTeleopCanvas,
    TopicSelector
  },
  data () {
    return {
      topicName: localStorage.getItem('TwistTeleop.topicName') || 'cmd_vel'
    }
  },
  watch: {
    topicName () {
      localStorage.setItem('TwistTeleop.topicName', this.topicName)
    }
  }
}
</script>

<style>
.twist-teleop-canvas-container {
  width: 100%;
  height: 300px;
}
</style>
