<template>
  <div class="card">
    <div class="card-header">
      Occupancy grid viewer
    </div>
    <topic-selector :initialTopicName="topicName" @updated="topicName = $event"></topic-selector>
    <div class="card-block" style="height: 300px;">
      <navigation-viewer :topicName="topicName" :ros="ros"></navigation-viewer>
    </div>
    <div class="card-footer text-muted">
      Listening to {{topicName}}
    </div>
  </div>
</template>

<script>
import TopicSelector from '@/components/ros/TopicSelector'
import NavigationViewer from '@/components/ros/NavigationViewer'
import localStorageUtil from '@/util/localStorage'
import ros from '@/services/ros'

export default {
  components: {
    TopicSelector,
    NavigationViewer
  },
  data () {
    return {
      ros: ros,
      topicName: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.topicName', 'map')
    }
  },
  watch: {
    topicName () {
      localStorage.setItem('NavigationViewer.topicName', this.topicName)
    }
  }
}
</script>
