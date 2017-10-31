<template>
  <div class="card">
    <div class="card-header">
      Occupancy grid viewer
    </div>
    <topic-selector label="Map topic" :initialTopicName="mapTopicName" @updated="mapTopicName = $event"></topic-selector>
    <topic-selector label="Goal topic" :initialTopicName="userGoalTopicName" @updated="userGoalTopicName = $event"></topic-selector>
    <frame-selector label="Robot frame" :initialFrameId="robotFrameId" @updated="robotFrameId = $event"></frame-selector>
    <div class="card-block" style="height: 600px;">
      <navigation-viewer :ros="ros" :mapTopicName="mapTopicName" :userGoalTopicName="userGoalTopicName" :robotFrameId="robotFrameId"></navigation-viewer>
    </div>
    <div class="card-footer text-muted">
      Map: {{mapTopicName}} - Robot frame: {{robotFrameId}} - Goal: {{userGoalTopicName}}
    </div>
  </div>
</template>

<script>
import TopicSelector from '@/components/ros/TopicSelector'
import FrameSelector from '@/components/ros/FrameSelector'
import NavigationViewer from '@/components/ros/NavigationViewer'
import localStorageUtil from '@/util/localStorage'
import ros from '@/services/ros'

export default {
  components: {
    TopicSelector,
    FrameSelector,
    NavigationViewer
  },
  data () {
    return {
      ros: ros,
      mapTopicName: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.mapTopicName', 'map'),
      userGoalTopicName: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.userGoalTopicName', 'goal'),
      robotFrameId: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.robotFrameId', 'base_link')
    }
  },
  watch: {
    mapTopicName () {
      localStorage.setItem('NavigationViewer.mapTopicName', this.mapTopicName)
    },
    userGoalTopicName () {
      localStorage.setItem('NavigationViewer.userGoalTopicName', this.userGoalTopicName)
    },
    robotFrameId () {
      localStorage.setItem('NavigationViewer.robotFrameId', this.robotFrameId)
    }
  }
}
</script>
