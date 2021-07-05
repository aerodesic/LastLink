function add_class(item, classname) {
    document.getElementById(item).classList.add(classname);
}

function remove_class(item, classname) {
    document.getElementById(item).classList.remove(classname);
}

//
// Rest command sender and processor.
//
var rest_command_queue = [];
var comm_stalled = false;
var xml_http = null;

//
// Send a rest command.
//     cmd is the url portion.
//     callback is to trigger a callback when reply is received. 
//     data is the json to be sent with the request.
//
// e.g.
//    send_rest_command("messages",
//                      function(results) {
//                        // Handle results
//                      },
//                      {
//                        <parameters to request>,
//                      },
//                      "post");
//
function send_rest_command(cmd, callback, data, request, payload) {
  callback = typeof(callback) !== 'undefined' ? callback : null;
  data = typeof(data) !== 'undefined' ? data : null;
  request = typeof(request) !== 'undefined' ? request : 'get';

  if (!comm_stalled) {
    queue_rest_command({ cmd: cmd, callback: callback, data: data, request: request, payload: payload });
  }
}

//
// Add the request to the queue.
//
// Return true if successful; false if duplicate.
//
function queue_rest_command(request) {

  // Look for duplicate and return false if found
  for (var index in msg_queue) {
    var elem = msg_queue[index] ;
    if (elem.cmd === request.cmd) {
      return false;
    }
  }

  // Not found - insert new one into queue
  msg_queue.push(request);

  // Activate if first one in the queue
  if (msg_queue.length === 1) {
    send_next_rest_command();
  }

  return true;
}

// <rc> is rest command.
function send_rest_command(rc) {
  if (typeof(rc) !== 'undefined') {
    var command = '/rest/' + rc.cmd;
    var callback = rc.callback;
    var data = rc.data;
    var request = rc.request;
    var payload = rc.payload;

    // Add escapes around the \ and : because they have special meaning inthe command process
    // command = encode_backslashes(command);

    // Encode it for transport over html
    // command = encodeURIComponent(command);

    // Encode as URL and send it
    var url = host_url + command + '?' + new Date().getTime().toString();

    xml_http = new XMLHttpRequest();

    xml_http.ontimeout = function() {
      if (++http_retry > MAX_HTTP_RETRIES) {
        //clearInterval(poll_timer);
        //poll_timer = null;
        if (!comm_stalled) {
          comm_stalled = true;
          alert("Communication failed - reload page to restart.");
          // Empty the queue
          msg_queue = [];
        }
      } else {
        // Send it again in a few milliseconds
        setTimeout(function() {
                     send_next_rest_command()
                   }, 500);
      }
    }

    xml_http.onreadystatechange = function() {
      if (xml_http.readyState === 4) {

        if (xml_http.status >= 200 && xml_http.status <= 299) {
          // If a callback is present, dispatch as event or function call
          if (callback !== null) {
            results = JSON.parse(xml_http.responseText);

            // If a callback is a string, send as an event
            if (typeof(callback) === 'function') {
              callback(results, payload);
            }
          }
          // Remove from queue
          msg_queue.splice(0, 1);
          http_retry = 0;
        } else {
          // an error occurred
          var text = xml_http.responseText;
        }

        // Clear queue and send next command in a few milliseconds
        setTimeout(function() {
          xml_http = null;
          send_next_rest_command();
        }, 50);
      }
    };

    xml_http.open(request, url, true);

    if (data !== null && typeof(data) !== 'string') {
      // If data is not a 'string' then assume JSON attachment rather than just string
      xml_http.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
      data = JSON.stringify(data);
    }

    xml_http.timeout = HTTP_RECV_TIMEOUT;
    xml_http.send(data);
  }
}


function send_next_rest_command() {
  if (!comm_stalled && !xml_http && msg_queue.length !== 0) {
    send_rest_command(msg_queue[0]);
  }
}

