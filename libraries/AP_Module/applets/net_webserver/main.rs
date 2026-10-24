// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/net_webserver.lua.
#![no_std]
#![no_main]

extern crate alloc;

use alloc::{string::{String, ToString}, vec::Vec};
use ap_rust_sdk::{enums::MavSeverity, Context, Error, Parameter, Result, Socket};
use core::fmt::Write;

ap_rust_sdk::module!(run);

const PARAM_TABLE_KEY: u8 = 47;
const REQUEST_MAX: usize = 4096;
const CLIENT_MAX: usize = 20;
const DIRECTORY_ENTRY_MAX: u16 = 2048;
const SERVER_VERSION: &str = "net_webserver 1.0";
const MAIN_PAGE: &str = r#"<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>ArduPilot</title></head>
<h2>ArduPilot Web Server</h2><body>
<div id="main"><ul><li><a href="mnt/">SDCard Access</a></li></ul></div>
<h2>Controller Status</h2><div id="board_status"></div>
<script>
function update_status() {
  fetch('/@DYNAMIC/board_status.shtml').then(r => r.text())
    .then(text => { document.getElementById('board_status').innerHTML = text; })
    .catch(() => {}).finally(() => setTimeout(update_status, 1000));
}
update_status();
</script>
</body></html>
"#;

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => {
                context.sleep_ms(1)?;
            }
            result => return result,
        }
    }
}

fn text(context: &Context<'_>, severity: i32, message: &str) -> Result {
    retry(context, || context.gcs().send_text(MavSeverity(severity), message))
}

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn value(parameter: &Parameter<'_, '_>) -> Result<f32> {
    parameter.get()?.filter(|number| number.is_finite()).ok_or(Error::Invalid)
}

fn bool_value(parameter: &Parameter<'_, '_>) -> Result<bool> {
    Ok(value(parameter)? != 0.0)
}

fn push_parameter(context: &Context<'_>, index: u8, name: &str, default: f32) -> Result {
    if retry(context, || context.param().add_param(PARAM_TABLE_KEY, index, name, default))? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

fn push_bytes(target: &mut Vec<u8>, bytes: &[u8]) -> Result {
    target.try_reserve(bytes.len()).map_err(|_| Error::NoMemory)?;
    target.extend_from_slice(bytes);
    Ok(())
}

fn header_as_bytes(mut header: String) -> Vec<u8> {
    header.push_str("Connection: close\r\n\r\n");
    header.into_bytes()
}

fn complete_response(code: u16, reason: &str, content_type: &str, body: &[u8]) -> Result<Vec<u8>> {
    let mut header = String::new();
    write!(header, "HTTP/1.1 {code} {reason}\r\nServer: {SERVER_VERSION}\r\n")
        .map_err(|_| Error::NoMemory)?;
    write!(header, "Content-Type: {content_type}\r\nContent-Length: {}\r\n", body.len())
        .map_err(|_| Error::NoMemory)?;
    let mut response = header_as_bytes(header);
    push_bytes(&mut response, body)?;
    Ok(response)
}

fn coordinate(value: i32) -> String {
    let whole = value / 10_000_000;
    let fraction = (i64::from(value) - i64::from(whole) * 10_000_000).abs();
    let mut result = String::new();
    // Match the Lua applet's latlon_str(): the fractional E7 remainder is
    // deliberately not zero padded (the existing autotest depends on this).
    let _ = write!(result, "{whole}.{fraction}");
    result
}

fn board_status(context: &Context<'_>) -> Result<Vec<u8>> {
    let firmware = retry(context, || context.fw_version().string())?;
    let hash = retry(context, || context.fw_version().hash())?;
    let armed = retry(context, || context.arming().is_armed())?;
    let location = retry(context, || context.ahrs().get_location())?;
    let seconds = context.time_us() / 1_000_000;
    let hours = seconds / 3600;
    let minutes = (seconds / 60) % 60;
    let mut body = String::new();
    write!(body, "<table><tr><td>Firmware</td><td>{}</td></tr>", firmware.as_str())
        .map_err(|_| Error::NoMemory)?;
    write!(body, "<tr><td>GIT Hash</td><td>{}</td></tr>", hash.as_str())
        .map_err(|_| Error::NoMemory)?;
    write!(body, "<tr><td>Uptime</td><td>{hours} hours {minutes} minutes {} seconds</td></tr>",
           seconds % 60).map_err(|_| Error::NoMemory)?;
    write!(body, "<tr><td>Arm Status</td><td>{}</td></tr>", if armed { "ARMED" } else { "DISARMED" })
        .map_err(|_| Error::NoMemory)?;
    if let Some(location) = location {
        let lat = coordinate(location.lat());
        let lon = coordinate(location.lng());
        write!(body, "<tr><td>AHRS Location</td><td>{lat} {lon} {:.1}m</td></tr>",
               location.alt() as f32 * 0.01).map_err(|_| Error::NoMemory)?;
        write!(body, "<tr><td>GPS Location</td><td>{lat} {lon} {:.1}m</td></tr>",
               location.alt() as f32 * 0.01).map_err(|_| Error::NoMemory)?;
    }
    body.push_str("</table>");
    Ok(body.into_bytes())
}

fn content_type(path: &str) -> &'static str {
    let extension = path.rsplit_once('.').map(|(_, extension)| extension).unwrap_or("");
    match extension {
        "htm" | "html" | "shtml" => "text/html;charset=UTF-8",
        "css" => "text/css",
        "csv" => "text/csv",
        "js" | "mjs" => "text/javascript",
        "json" => "application/json",
        "lua" => "text/x-lua",
        "py" => "text/x-python",
        "svg" => "image/svg+xml",
        "txt" => "text/plain",
        "gif" => "image/gif",
        "jpeg" | "jpg" => "image/jpeg",
        "png" => "image/png",
        _ => "application/octet-stream",
    }
}

fn filesystem_path(uri: &str) -> String {
    if uri == "/mnt" || uri == "/mnt/" {
        "/".to_string()
    } else if let Some(path) = uri.strip_prefix("/mnt/") {
        path.to_string()
    } else if let Some(path) = uri.strip_prefix("/@") {
        let mut result = String::from("@");
        result.push_str(path);
        result
    } else {
        uri.to_string()
    }
}

fn directory_page(context: &Context<'_>, path: &str) -> Result<Vec<u8>> {
    let mut body = String::new();
    write!(body, "<!DOCTYPE HTML><html><head><title>Index of {path}</title></head>")
        .map_err(|_| Error::NoMemory)?;
    write!(body, "<body><h1>Index of {path}</h1><table>")
        .map_err(|_| Error::NoMemory)?;
    body.push_str("<tr><th align=\"left\">Name</th><th align=\"left\">Size</th></tr>");
    for index in 0..DIRECTORY_ENTRY_MAX {
        let Some(entry) = retry(context, || context.fs().dir_entry(path, index))? else {
            break;
        };
        if entry.as_str() == "." {
            continue;
        }
        let mut full = if path == "/" { String::new() } else { path.to_string() };
        if !full.ends_with('/') {
            full.push('/');
        }
        full.push_str(entry.as_str());
        let stat = retry(context, || context.fs().stat(&full))?;
        let directory = stat.map(|item| item.is_directory).unwrap_or(false);
        let size = stat.map(|item| item.size).unwrap_or(0);
        write!(body, "<tr><td><a href=\"{}{}\">{}{}</a></td><td>{size}</td></tr>",
               entry.as_str(), if directory { "/" } else { "" }, entry.as_str(),
               if directory { "/" } else { "" }).map_err(|_| Error::NoMemory)?;
    }
    body.push_str("</table></body></html>");
    Ok(body.into_bytes())
}

enum Body {
    None,
    File { path: String, offset: u32, pending: Vec<u8>, sent: usize },
    Sendfile { path: String },
}

struct Client {
    socket: Socket,
    request: Vec<u8>,
    response: Vec<u8>,
    response_sent: usize,
    body: Body,
    start_ms: u64,
    ready: bool,
    closed: bool,
}

impl Client {
    fn new(socket: Socket, start_ms: u64) -> Self {
        Self { socket, request: Vec::new(), response: Vec::new(), response_sent: 0,
               body: Body::None, start_ms, ready: false, closed: false }
    }

    fn close(&mut self, context: &Context<'_>) {
        if !self.closed {
            let _ = self.socket.close(context);
            self.closed = true;
        }
    }

    fn read_request(&mut self, context: &Context<'_>, timeout_ms: u64) -> Result {
        let mut buffer = [0u8; 1024];
        match self.socket.recv(context, &mut buffer)? {
            Some(0) => self.close(context),
            Some(length) => {
                if self.request.len() + length > REQUEST_MAX {
                    self.close(context);
                } else {
                    push_bytes(&mut self.request, &buffer[..length])?;
                    self.ready = self.request.windows(4).any(|window| window == b"\r\n\r\n");
                }
            }
            None => {
                let now_ms = context.time_us() / 1000;
                if now_ms.saturating_sub(self.start_ms) > timeout_ms ||
                    !self.socket.is_connected(context)? {
                    self.close(context);
                }
            }
        }
        Ok(())
    }

    fn prepare(&mut self, context: &Context<'_>, sendfile_min: u32) -> Result {
        let request = core::str::from_utf8(&self.request).map_err(|_| Error::Invalid)?;
        let first = request.split("\r\n").next().ok_or(Error::Invalid)?;
        let mut words = first.split_whitespace();
        let operation = words.next().unwrap_or("");
        let uri = words.next().unwrap_or("");
        let protocol = words.next().unwrap_or("");
        if operation != "GET" || (protocol != "HTTP/1.0" && protocol != "HTTP/1.1") {
            self.response = complete_response(400, "Bad Request", "text/plain", b"Bad Request\n")?;
            return Ok(());
        }
        if uri == "/" {
            self.response = complete_response(200, "OK", "text/html;charset=UTF-8", MAIN_PAGE.as_bytes())?;
            return Ok(());
        }
        if uri == "/@DYNAMIC/board_status.shtml" {
            let body = board_status(context)?;
            self.response = complete_response(200, "OK", "text/html;charset=UTF-8", &body)?;
            return Ok(());
        }
        let path = filesystem_path(uri);
        let stat = retry(context, || context.fs().stat(&path))?;
        if let Some(stat) = stat {
            if stat.is_directory {
                let body = directory_page(context, &path)?;
                self.response = complete_response(200, "OK", "text/html;charset=UTF-8", &body)?;
            } else {
                let mut header = String::new();
                write!(header,
                       "HTTP/1.1 200 OK\r\nServer: {SERVER_VERSION}\r\nContent-Type: {}\r\n",
                       content_type(&path)).map_err(|_| Error::NoMemory)?;
                let known_size = !path.starts_with('@');
                if known_size {
                    write!(header, "Content-Length: {}\r\n", stat.size).map_err(|_| Error::NoMemory)?;
                }
                self.response = header_as_bytes(header);
                self.body = if known_size && sendfile_min > 0 && stat.size >= sendfile_min {
                    Body::Sendfile { path }
                } else {
                    Body::File { path, offset: 0, pending: Vec::new(), sent: 0 }
                };
            }
        } else {
            self.response = complete_response(404, "Not Found", "text/plain", b"Not Found\n")?;
        }
        Ok(())
    }

    fn send_buffer(socket: &Socket, context: &Context<'_>, buffer: &[u8], offset: &mut usize) -> Result<bool> {
        if *offset >= buffer.len() {
            return Ok(true);
        }
        let end = (*offset + 1024).min(buffer.len());
        if let Some(sent) = socket.send(context, &buffer[*offset..end])? {
            if sent == 0 {
                return Err(Error::Host);
            }
            *offset += sent;
        }
        Ok(*offset == buffer.len())
    }

    fn update_body(&mut self, context: &Context<'_>) -> Result {
        match &mut self.body {
            Body::None => self.close(context),
            Body::Sendfile { path } => {
                if self.socket.sendfile(context, path)? {
                    self.closed = true;
                } else {
                    self.body = Body::File { path: path.clone(), offset: 0, pending: Vec::new(), sent: 0 };
                }
            }
            Body::File { path, offset, pending, sent } => {
                if *sent < pending.len() {
                    if Self::send_buffer(&self.socket, context, pending, sent)? {
                        *offset = offset.checked_add(pending.len() as u32).ok_or(Error::Invalid)?;
                        pending.clear();
                        *sent = 0;
                    }
                    return Ok(());
                }
                let mut buffer = [0u8; 1024];
                let Some(length) = retry(context, || context.fs().read_at(path, *offset, &mut buffer))? else {
                    self.close(context);
                    return Ok(());
                };
                if length == 0 {
                    self.close(context);
                    return Ok(());
                }
                push_bytes(pending, &buffer[..length])?;
                if Self::send_buffer(&self.socket, context, pending, sent)? {
                    *offset = offset.checked_add(pending.len() as u32).ok_or(Error::Invalid)?;
                    pending.clear();
                    *sent = 0;
                }
            }
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>, timeout_ms: u64, sendfile_min: u32) -> Result {
        if self.closed {
            return Ok(());
        }
        if !self.ready {
            self.read_request(context, timeout_ms)?;
            if !self.ready || self.closed {
                return Ok(());
            }
            self.prepare(context, sendfile_min)?;
        }
        if !Self::send_buffer(&self.socket, context, &self.response, &mut self.response_sent)? {
            return Ok(());
        }
        self.update_body(context)
    }
}

fn run(context: &Context<'_>) -> Result {
    if !retry(context, || context.param().add_table(PARAM_TABLE_KEY, "WEB_", 6))? {
        return Err(Error::Host);
    }
    push_parameter(context, 1, "ENABLE", 1.0)?;
    push_parameter(context, 2, "BIND_PORT", 8080.0)?;
    push_parameter(context, 3, "DEBUG", 0.0)?;
    push_parameter(context, 4, "BLOCK_SIZE", 10_240.0)?;
    push_parameter(context, 5, "TIMEOUT", 2.0)?;
    push_parameter(context, 6, "SENDFILE_MIN", 100_000.0)?;
    let enabled = bind(context, "WEB_ENABLE")?;
    let bind_port = bind(context, "WEB_BIND_PORT")?;
    let timeout = bind(context, "WEB_TIMEOUT")?;
    let sendfile_min = bind(context, "WEB_SENDFILE_MIN")?;
    if !bool_value(&enabled)? {
        text(context, 6, "WebServer: disabled")?;
        return Ok(());
    }
    let port_value = value(&bind_port)?;
    if !(1.0..=65_535.0).contains(&port_value) {
        return Err(Error::Invalid);
    }
    let port = port_value as u16;
    let mut message = String::new();
    write!(message, "WebServer: starting on port {port}").map_err(|_| Error::NoMemory)?;
    text(context, 6, &message)?;
    let listener = context.network().open(false)?;
    if !listener.bind(context, "0.0.0.0", port)? {
        message.clear();
        write!(message, "WebServer: failed to bind to TCP {port}").map_err(|_| Error::NoMemory)?;
        text(context, 3, &message)?;
        return Ok(());
    }
    if !listener.listen(context, CLIENT_MAX as u8)? {
        text(context, 3, "WebServer: failed to listen")?;
        return Ok(());
    }
    listener.set_blocking(context, false)?;
    let mut clients = Vec::<Client>::new();
    loop {
        while clients.len() < CLIENT_MAX && listener.pollin(context, 0)? {
            let Some(socket) = listener.accept(context)? else { break };
            socket.set_blocking(context, false)?;
            clients.try_reserve(1).map_err(|_| Error::NoMemory)?;
            clients.push(Client::new(socket, context.time_us() / 1000));
        }
        let timeout_ms = (value(&timeout)?.clamp(0.1, 60.0) * 1000.0) as u64;
        let threshold = value(&sendfile_min)?.max(0.0) as u32;
        for client in clients.iter_mut() {
            if let Err(error) = client.update(context, timeout_ms, threshold) {
                client.close(context);
                if error == Error::NoMemory {
                    return Err(error);
                }
            }
        }
        clients.retain(|client| !client.closed);
        context.sleep_ms(5)?;
    }
}
