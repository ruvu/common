<template>
  <div>
    <div v-if="castGoal">
      <h1 v-text="castGoal.message"></h1>
    </div>
    <div v-if="queryGoal">
      <b-list-group>
        <b-list-group-item v-for="s in queryGoal.example_sentences" :key="s.id"
                           v-text="s"
                           href="#"
                           @click="queryServer.setSucceeded({sentence: s}); queryGoal = null">
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
    castActionName: {
      required: true,
      type: String
    },
    queryActionName: {
      required: true,
      type: String
    }
  },
  data () {
    return {
      castServer: null,
      castGoal: null,
      queryServer: null,
      queryGoal: null
    }
  },
  created () {
    this.readvertise()
  },
  watch: {
    castActionName () {
      this.readvertise()
    },
    queryActionName () {
      this.readvertise()
    }
  },
  methods: {
    readvertise () {
      this.castServer = new ROSLIB.SimpleActionServer({
        ros: this.ros,
        serverName: this.castActionName,
        actionName: 'hmi_msgs/CastAction'
      })
      this.castServer.on('goal', (goal) => {
        console.log('goal cast', goal)
        this.castGoal = goal
      })
      this.castServer.on('cancel', () => {
        console.log('cancel')
        this.castServer.setPreempted()
        this.castGoal = null
      })
      this.queryServer = new ROSLIB.SimpleActionServer({
        ros: this.ros,
        serverName: this.queryActionName,
        actionName: 'hmi_msgs/QueryAction'
      })
      this.queryServer.on('goal', (goal) => {
        console.log('goal query', goal)
        this.queryGoal = goal
      })
      this.queryServer.on('cancel', () => {
        console.log('cancel query')
        this.queryServer.setPreempted()
        this.queryGoal = null
      })
    }
  }
}
</script>
