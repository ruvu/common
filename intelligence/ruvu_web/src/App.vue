<template>
  <div id="app">
    <b-navbar toggleable="md"  type="light" variant="light" :sticky="true" fixed="top">
      <b-nav-toggle target="nav_collapse"></b-nav-toggle>

      <b-collapse is-nav id="nav_collapse">
        <b-nav is-nav-bar>
          <router-link tag="li" to="/" exact>
            <a class="nav-link">Twist teleop</a>
          </router-link>
          <router-link tag="li" to="/string_publisher" exact>
            <a class="nav-link">String publisher</a>
          </router-link>
          <router-link tag="li" to="/hmi_cast_client" exact>
            <a class="nav-link">HMI cast client</a>
          </router-link>
          <router-link tag="li" to="/hmi_query_client" exact>
            <a class="nav-link">HMI query client</a>
          </router-link>
          <router-link tag="li" to="/hmi_server" exact>
            <a class="nav-link">HMI server</a>
          </router-link>
          <router-link tag="li" to="/navigation_viewer" exact>
            <a class="nav-link">Navigation viewer</a>
          </router-link>
        </b-nav>
      </b-collapse>

      <!-- Right aligned nav items -->
      <b-nav is-nav-bar class="ml-auto">
        <connection-status-button :ros="ros"></connection-status-button>
      </b-nav>
    </b-navbar>

    <div class="container" id="main">
      <router-view></router-view>
    </div>
  </div>
</template>

<script>
import ConnectionStatusButton from '@/components/ros/ConnectionStatusButton'
import ros from '@/services/ros'

export default {
  components: {
    ConnectionStatusButton
  },
  data () {
    return {
      ros: ros
    }
  },
  name: 'app'
}
</script>

<style>
#main {
  padding-top: 20px
}
.btn {
  cursor: pointer;
}
</style>
