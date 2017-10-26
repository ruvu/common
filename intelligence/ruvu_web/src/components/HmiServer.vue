<template>
  <div class="card">
    <div class="card-header">
      HMI server
    </div>
    <action-selector :initialActionName="castActionName" @updated="castActionName = $event" messageType="hmi_msgs/CastAction"></action-selector>
    <action-selector :initialActionName="queryActionName" @updated="queryActionName = $event" messageType="hmi_msgs/QueryAction"></action-selector>
    <div class="card-block">
      <hmi-action-server :castActionName="castActionName" :queryActionName="queryActionName" :ros="ros"></hmi-action-server>
    </div>
    <div class="card-footer text-muted">
      Cast action {{castActionName}} - Query action {{queryActionName}}
    </div>
  </div>
</template>

<script>
import ActionSelector from '@/components/ros/ActionSelector'
import HmiActionServer from '@/components/ros/HmiActionServer'
import localStorageUtil from '@/util/localStorage'
import ros from '@/services/ros'

export default {
  components: {
    ActionSelector,
    HmiActionServer
  },
  data () {
    return {
      ros: ros,
      castActionName: localStorageUtil.getLocalStorageWithDefault('HmiServer.castActionName', 'hmi/cast/web'),
      queryActionName: localStorageUtil.getLocalStorageWithDefault('HmiServer.queryActionName', 'hmi/query/web')
    }
  },
  watch: {
    castActionName () {
      localStorage.setItem('HmiCast.castActionName', this.castActionName)
    },
    queryActionName () {
      localStorage.setItem('HmiCast.queryActionName', this.queryActionName)
    }
  }
}
</script>
