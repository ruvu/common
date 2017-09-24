function getLocalStorageWithDefault (name, defaultValue) {
  var value = localStorage.getItem(name)
  if (value !== undefined && typeof value === typeof defaultValue) {
    return value
  }
  return defaultValue
}

export default {getLocalStorageWithDefault}
