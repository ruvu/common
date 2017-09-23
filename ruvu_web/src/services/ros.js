import ROSLIB from 'roslib'
import queryString from 'query-string'

// reconnect timeout in ms
const RECONNECT_TIMEOUT = 5000

// Connection constructor
class Connection extends ROSLIB.Ros {

  constructor () {
    super({encoding: 'ascii'})

    const host = queryString.parse(location.search)['host'] || 'localhost'
    this.url = `ws://${host}:9090`

    this.status = {
      message: 'closed',
      lifetime: 0
    }

    this.on('connection', () => {
      console.log('Connected')
      this.status.message = 'connected'
    })
    this.on('close', () => {
      setTimeout(this._connect(), RECONNECT_TIMEOUT)
      console.log('Connection closed')
      this.status.message = 'closed'
    })
    this.on('error', () => {
      this.status.message = 'error'
    })

    this._connect()
  }

  _connect () {
    console.log(`connecting to ${this.url}`)
    this.connect(this.url)
    this.status.message = 'connecting'
  }
}

export default new Connection()
