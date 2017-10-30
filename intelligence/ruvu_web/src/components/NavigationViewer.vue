<template>
  <div class="card">
    <div class="card-header">
      Occupancy grid viewer
    </div>
    <topic-selector label="Map topic" :initialTopicName="mapTopicName" @updated="mapTopicName = $event"></topic-selector>
    <topic-selector label="Polygon topic" :initialTopicName="polygonTopicName" @updated="polygonTopicName = $event"></topic-selector>
    <div class="card-block" style="height: 600px;">
      <navigation-viewer :ros="ros" :mapTopicName="mapTopicName" :polygonTopicName="polygonTopicName"></navigation-viewer>
    </div>
    <div class="card-footer text-muted">
      Map: {{mapTopicName}} - Polygon: {{polygonTopicName}}
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
      mapTopicName: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.mapTopicName', 'map'),
      polygonTopicName: localStorageUtil.getLocalStorageWithDefault('NavigationViewer.polygonTopicName', 'polygon')
    }
  },
  watch: {
    mapTopicName () {
      localStorage.setItem('NavigationViewer.mapTopicName', this.mapTopicName)
    },
    polygonTopicName () {
      localStorage.setItem('NavigationViewer.polygonTopicName', this.polygonTopicName)
    }
  }
}
</script>
