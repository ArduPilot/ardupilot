import Foundation

/// Common protocol for all MAVLink entities which describes types
/// metadata properties.
public protocol MAVLinkEntity: CustomStringConvertible, CustomDebugStringConvertible {
    
    /// Original MAVLink enum name (from declarations xml)
    static var typeName: String { get }
    
    /// Compact type description
    static var typeDescription: String { get }
    
    /// Verbose type description
    static var typeDebugDescription: String { get }
}

// MARK: - Enumeration protocol

/// Enumeration protocol description with common for all MAVLink enums
/// properties requirements.
public protocol Enumeration: RawRepresentable, Equatable, MAVLinkEntity {
    
    /// Array with all members of current enum
    static var allMembers: [Self] { get }
    
    // Array with `Name` - `Description` tuples (values from declarations xml file)
    static var membersDescriptions: [(String, String)] { get }
    
    /// `ENUM_END` flag for checking if enum case value is valid
    static var enumEnd: UInt { get }
    
    /// Original MAVLinks enum member name (as declared in definition's xml file)
    var memberName: String { get }
    
    /// Specific member description from definitions xml
    var memberDescription: String { get }
}

/// Enumeration protocol default behaviour implementation.
extension Enumeration {
    public static var typeDebugDescription: String {
        let cases = allMembers.map({ $0.debugDescription }).joined(separator: "\\n\\t")
        return "Enum \(typeName): \(typeDescription)\\nMembers:\\n\\t\(cases)"
    }
    
    public var description: String {
        return memberName
    }
    
    public var debugDescription: String {
        return "\(memberName): \(memberDescription)"
    }
    
    public var memberName: String {
        return Self.membersDescriptions[Self.allMembers.index(of: self)!].0
    }
    
    public var memberDescription: String {
        return Self.membersDescriptions[Self.allMembers.index(of: self)!].1
    }
}

// MARK: - MAVLinkBitmask protocol

public protocol MAVLinkBitmask: OptionSet, MAVLinkEntity {
    /// Array with all members of current bitmask
    static var allMembers: [Self.Element] { get }

    // Array with `Name` - `Description` tuples (values from declarations xml file)
    static var membersDescriptions: [(String, String)] { get }

    /// `ENUM_END` flag for checking if enum case value is valid
    static var enumEnd: UInt { get }

    /// Original MAVLinks enum member name (as declared in definition's xml file)
    var usedMemberName: [String] { get }

    /// Specific member description from definitions xml
    var usedMemberDescriptions: [String] { get }
}

/// MAVLinkBitmask protocol default behaviour implementation.
extension MAVLinkBitmask {
    public static var typeDebugDescription: String {
        let cases = membersDescriptions.map { "\($0.0): \($0.1)" }.joined(separator: "\\n\\t")
        return "Bitmask \(typeName): \(typeDescription)\\nMembers:\\n\\t\(cases)"
    }

    public var description: String {
        return metadataForUsedMembers().map { $0.1 }.joined(separator:", ")
    }

    public var debugDescription: String {
        let usedValuesExplained = metadataForUsedMembers().map {
            "\($0.0): \($0.1)"
            }.joined(separator: "\n")

        return usedValuesExplained
    }

    public var usedMemberName: [String] {
        return metadataForUsedMembers().map { $0.0 }
    }

    public var usedMemberDescriptions: [String] {
        return metadataForUsedMembers().map { $0.1 }
    }

    private func metadataForUsedMembers() -> [(String, String)] {
        return zip(Self.allMembers, Self.membersDescriptions).filter {
                self.contains($0.0)
            }.map {
                $0.1
            }
    }
}

// MARK: - Message protocol

/// Message field definition tuple.
public typealias FieldDefinition = (name: String, offset: Int, type: String, length: UInt, description: String)

/// Message protocol describes all common MAVLink messages properties and
/// methods requirements.
public protocol Message: MAVLinkEntity {
    static var id: UInt8 { get }
    
    static var payloadLength: UInt8 { get }
    
    /// Array of tuples with field definition info
    static var fieldDefinitions: [FieldDefinition] { get }
    
    /// All field's names and values of current Message
    var allFields: [(String, Any)] { get }
    
    /// Initialize Message from received data.
    ///
    /// - parameter data: Data to decode.
    ///
    /// - throws: Throws `ParseError` or `ParseEnumError` if any parsing errors
    /// occur.
    init(data: Data) throws
    
    /// Returns `Data` representation of current `Message` struct guided
    /// by format from `fieldDefinitions`.
    ///
    /// - throws: Throws `PackError` if any of message fields do not comply
    /// format from `fieldDefinitions`.
    ///
    /// - returns: Receiver's `Data` representation
    func pack() throws -> Data
}

/// Message protocol default behaviour implementation.
extension Message {
    public static var payloadLength: UInt8 {
        return messageLengths[id] ?? Packet.Constant.maxPayloadLength
    }
    
    public static var typeDebugDescription: String {
        let fields = fieldDefinitions.map({ "\($0.name): \($0.type): \($0.description)" }).joined(separator: "\n\t")
        return "Struct \(typeName): \(typeDescription)\nFields:\n\t\(fields)"
    }
    
    public var description: String {
        let describeField: ((String, Any)) -> String = { (arg) in
            let (name, value) = arg
            let valueString = value is String ? "\"\(value)\"" : value
            return "\(name): \(valueString)"
        }
        let fieldsDescription = allFields.map(describeField).joined(separator: ", ")
        return "\(type(of: self))(\(fieldsDescription))"
    }
    
    public var debugDescription: String {
        let describeFieldVerbose: ((String, Any)) -> String = { (arg) in
            let (name, value) = arg
            let valueString = value is String ? "\"\(value)\"" : value
            let (_, _, _, _, description) = Self.fieldDefinitions.filter { $0.name == name }.first!
            return "\(name) = \(valueString) : \(description)"
        }
        let fieldsDescription = allFields.map(describeFieldVerbose).joined(separator: "\n\t")
        return "\(Self.typeName): \(Self.typeDescription)\nFields:\n\t\(fieldsDescription)"
    }
    
    public var allFields: [(String, Any)] {
        var result: [(String, Any)] = []
        let mirror = Mirror(reflecting: self)
        for case let (label?, value) in mirror.children {
            result.append((label, value))
        }
        return result
    }
}

// MARK: - Type aliases

public typealias Channel = UInt8

// MARK: - Errors

public protocol MAVLinkError: Error, CustomStringConvertible, CustomDebugStringConvertible { }

// MARK: Parsing error enumeration

/// Parsing errors
public enum ParseError: MAVLinkError {
    
    /// Size of expected number is larger than receiver's data length.
    /// - offset:     Expected number offset in received data.
    /// - size:       Expected number size in bytes.
    /// - upperBound: The number of bytes in the data.
    case valueSizeOutOfBounds(offset: Int, size: Int, upperBound: Int)
    
    /// Data contains non ASCII characters.
    /// - offset: String offset in received data.
    /// - length: Expected length of string to read.
    case invalidStringEncoding(offset: Int, length: Int)
    
    /// Length check of payload for known `messageId` did fail.
    /// - messageId:      Id of expected `Message` type.
    /// - receivedLength: Received payload length.
    /// - properLength:   Expected payload length for `Message` type.
    case invalidPayloadLength(messageId: UInt8, receivedLength: UInt8, expectedLength: UInt8)
    
    /// Received `messageId` was not recognized so we can't create appropriate
    /// `Message`.
    /// - messageId: Id of the message that was not found in the known message
    /// list (`messageIdToClass` array).
    case unknownMessageId(messageId: UInt8)
    
    /// Checksum check failed. Message id is known but calculated CRC bytes
    /// do not match received CRC value.
    /// - messageId: Id of expected `Message` type.
    case badCRC(messageId: UInt8)
}

extension ParseError {
    
    /// Textual representation used when written to output stream.
    public var description: String {
        switch self {
        case .valueSizeOutOfBounds:
            return "ParseError.valueSizeOutOfBounds"
        case .invalidStringEncoding:
            return "ParseError.invalidStringEncoding"
        case .invalidPayloadLength:
            return "ParseError.invalidPayloadLength"
        case .unknownMessageId:
            return "ParseError.unknownMessageId"
        case .badCRC:
            return "ParseError.badCRC"
        }
    }
    
    /// Debug textual representation used when written to output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .valueSizeOutOfBounds(offset, size, upperBound):
            return "ParseError.valueSizeOutOfBounds(offset: \(offset), size: \(size), upperBound: \(upperBound))"
        case let .invalidStringEncoding(offset, length):
            return "ParseError.invalidStringEncoding(offset: \(offset), length: \(length))"
        case let .invalidPayloadLength(messageId, receivedLength, expectedLength):
            return "ParseError.invalidPayloadLength(messageId: \(messageId), receivedLength: \(receivedLength), expectedLength: \(expectedLength))"
        case let .unknownMessageId(messageId):
            return "ParseError.unknownMessageId(messageId: \(messageId))"
        case let .badCRC(messageId):
            return "ParseError.badCRC(messageId: \(messageId))"
        }
    }
}

// MARK: Parsing enumeration error

/// Special error type for returning Enum parsing errors with details in associated
/// values (types of these values are not compatible with `ParseError` enum).
public enum ParseEnumError<T: RawRepresentable>: MAVLinkError {
    
    /// Enumeration case with `rawValue` at `valueOffset` was not found in
    /// `enumType` enumeration.
    /// - enumType: Type of expected enumeration.
    /// - rawValue: Raw value that was not found in `enumType`.
    /// - valueOffset: Value offset in received payload data.
    case unknownValue(enumType: T.Type, rawValue: T.RawValue, valueOffset: Int)
}

extension ParseEnumError {
    
    /// Textual representation used when written to the output stream.
    public var description: String {
        switch self {
        case .unknownValue:
            return "ParseEnumError.unknownValue"
        }
    }
    
    /// Debug textual representation used when written to the output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .unknownValue(enumType, rawValue, valueOffset):
            return "ParseEnumError.unknownValue(enumType: \(enumType), rawValue: \(rawValue), valueOffset: \(valueOffset))"
        }
    }
}

// MARK: Packing errors

/// Errors that can occur while packing `Message` for sending.
public enum PackError: MAVLinkError {
    
    /// Size of received value (together with offset) is out of receiver's length.
    /// - offset:     Expected value offset in payload.
    /// - size:       Provided field value size in bytes.
    /// - upperBound: Available payload length.
    case valueSizeOutOfBounds(offset: Int, size: Int, upperBound: Int)
    
    /// Length check for provided field value did fail.
    /// - offset:              Expected value offset in payload.
    /// - providedValueLength: Count of elements (characters) in provided value.
    /// - allowedLength:       Maximum number of elements (characters) allowed in field.
    case invalidValueLength(offset: Int, providedValueLength: Int, allowedLength: Int)
    
    /// String field contains non ASCII characters.
    /// - offset: Expected value offset in payload.
    /// - string: Original string.
    case invalidStringEncoding(offset: Int, string: String)
    
    /// CRC extra byte not found for provided `messageId` type.
    /// - messageId: Id of message type.
    case crcExtraNotFound(messageId: UInt8)
    
    /// Packet finalization process failed due to `message` absence.
    case messageNotSet
}

extension PackError {
    
    /// Textual representation used when written to the output stream.
    public var description: String {
        switch self {
        case .valueSizeOutOfBounds:
            return "PackError.valueSizeOutOfBounds"
        case .invalidValueLength:
            return "PackError.invalidValueLength"
        case .invalidStringEncoding:
            return "PackError.invalidStringEncoding"
        case .crcExtraNotFound:
            return "PackError.crcExtraNotFound"
        case .messageNotSet:
            return "PackError.messageNotSet"
        }
    }
    
    /// Debug textual representation used when written to the output stream, which
    /// includes all associated values and their labels.
    public var debugDescription: String {
        switch self {
        case let .valueSizeOutOfBounds(offset, size, upperBound):
            return "PackError.valueSizeOutOfBounds(offset: \(offset), size: \(size), upperBound: \(upperBound))"
        case let .invalidValueLength(offset, providedValueLength, allowedLength):
            return "PackError.invalidValueLength(offset: \(offset), providedValueLength: \(providedValueLength), allowedLength: \(allowedLength))"
        case let .invalidStringEncoding(offset, string):
            return "PackError.invalidStringEncoding(offset: \(offset), string: \(string))"
        case let .crcExtraNotFound(messageId):
            return "PackError.crcExtraNotFound(messageId: \(messageId))"
        case .messageNotSet:
            return "PackError.messageNotSet"
        }
    }
}

// MARK: - Delegate protocol

/// Alternative way to receive parsed Messages, finalized packet's data and all
/// errors is to implement this protocol and set as `MAVLink`'s delegate.
public protocol MAVLinkDelegate: class {
    
    /// Called when MAVLink packet is successfully received, payload length
    /// and CRC checks are passed.
    ///
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter channel: Channel on which `packet` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didReceive(packet: Packet, on channel: Channel, via link: MAVLink)
    
    /// Packet receiving failed due to `InvalidPayloadLength` or `BadCRC` error.
    ///
    /// - parameter packet:    Partially received `Packet`.
    /// - parameter error:     Error that  occurred while receiving `data`
    /// (`InvalidPayloadLength` or `BadCRC` error).
    /// - parameter channel:   Channel on which `packet` was received.
    /// - parameter link:      `MAVLink` object that received `data`.
    func didFailToReceive(packet: Packet?, with error: MAVLinkError, on channel: Channel, via link: MAVLink)
    
    /// Called when received data was successfully parsed into appropriate
    /// `message` structure.
    ///
    /// - parameter message: Successfully parsed `Message`.
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter channel: Channel on which `message` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didParse(message: Message, from packet: Packet, on channel: Channel, via link: MAVLink)
    
    /// Called when `packet` completely received but `MAVLink` was not able to
    /// finish `Message` processing due to unknown `messageId` or type validation
    /// errors.
    ///
    /// - parameter packet:  Completely received `Packet`.
    /// - parameter error:   Error that  occurred while parsing `packet`'s
    /// payload into `Message`.
    /// - parameter channel: Channel on which `message` was received.
    /// - parameter link:    `MAVLink` object that handled `packet`.
    func didFailToParseMessage(from packet: Packet, with error: MAVLinkError, on channel: Channel, via link: MAVLink)
    
    /// Called when message is finalized and ready for sending to aircraft.
    ///
    /// - parameter message: Message to be sent.
    /// - parameter data:    Compiled data that represents `message`.
    /// - parameter channel: Channel on which `message` should be sent.
    /// - parameter link:    `MAVLink` object that handled `message`.
    func didFinalize(message: Message, from packet: Packet, to data: Data, on channel: Channel, in link: MAVLink)
}

// MARK: - Classes implementations

/// Main MAVLink class, performs `Packet` receiving, recognition, validation,
/// `Message` structure creation and `Message` packing, finalizing for sending.
/// Also returns errors through delegation if any errors occurred.
/// - warning: Supports only 1.0 version of the MAVlink wire protocol.
public class MAVLink {
    
    /// States for the parsing state machine.
    enum ParseState {
        case uninit
        case idle
        case gotStx
        case gotSequence
        case gotLength
        case gotSystemId
        case gotComponentId
        case gotMessageId
        case gotPayload
        case gotCRC1
        case gotBadCRC1
    }
    
    enum Framing: UInt8 {
        case incomplete = 0
        case ok = 1
        case badCRC = 2
    }
    
    /// Storage for MAVLink parsed packets count, states and errors statistics.
    class Status {
        
        /// Number of received packets
        var packetReceived: Framing = .incomplete
        
        /// Number of parse errors
        var parseError: UInt8 = 0
        
        /// Parsing state machine
        var parseState: ParseState = .uninit
        
        /// Sequence number of the last received packet
        var currentRxSeq: UInt8 = 0
        
        /// Sequence number of the last sent packet
        var currentTxSeq: UInt8 = 0
        
        /// Received packets
        var packetRxSuccessCount: UInt16 = 0
        
        /// Number of packet drops
        var packetRxDropCount: UInt16 = 0
    }
    
    /// MAVLink Packets and States buffers
    let channelBuffers = (0 ..< Channel.max).map({ _ in Packet() })
    let channelStatuses = (0 ..< Channel.max).map({ _ in Status() })
    
    /// Object to pass received packets, messages, errors, finalized data to.
    public weak var delegate: MAVLinkDelegate?
    
    /// Enable this option to check the length of each message. This allows
    /// invalid messages to be caught much sooner. Use it if the transmission
    /// medium is prone to missing (or extra) characters (e.g. a radio that
    /// fades in and out). Use only if the channel will contain message
    /// types listed in the headers.
    public var checkMessageLength = true
    
    /// Use one extra CRC that is added to the message CRC to detect mismatches
    /// in the message specifications. This is to prevent that two devices using
    /// different message versions incorrectly decode a message with the same
    /// length. Defined as `let` as we support only the latest version (1.0) of
    /// the MAVLink wire protocol.
    public let crcExtra = true
    
    public init() { }
    
    /// This is a convenience function which handles the complete MAVLink
    /// parsing. The function will parse one byte at a time and return the
    /// complete packet once it could be successfully decoded. Checksum and
    /// other failures will be delegated to `delegate`.
    ///
    /// - parameter char:    The char to parse.
    /// - parameter channel: Id of the current channel. This allows to parse
    /// different channels with this function. A channel is not a physical
    /// message channel like a serial port, but a logic partition of the
    /// communication streams in this case.
    ///
    /// - returns: Returns `nil` if packet could be decoded at the moment,
    /// the `Packet` structure else.
    public func parse(char: UInt8, channel: Channel) -> Packet? {
        
        /// Function to check if current char is Stx byte. If current char is
        /// STX, modifies current rxpack and status.
        func handleSTX(char: UInt8, rxpack: Packet, status: Status) {
            if char == Packet.Constant.packetStx {
                rxpack.length = 0
                rxpack.channel = channel
                rxpack.magic = char
                rxpack.checksum.start()
                status.parseState = .gotStx
            }
        }
        
        let rxpack = channelBuffers[Int(channel)]
        let status = channelStatuses[Int(channel)]
        
        status.packetReceived = .incomplete
        
        switch status.parseState {
        case .uninit, .idle:
            handleSTX(char: char, rxpack: rxpack, status: status)
            
        case .gotStx:
            rxpack.length = char
            rxpack.payload.count = 0
            rxpack.checksum.accumulate(char)
            status.parseState = .gotLength
            
        case .gotLength:
            rxpack.sequence = char
            rxpack.checksum.accumulate(char)
            status.parseState = .gotSequence
            
        case .gotSequence:
            rxpack.systemId = char
            rxpack.checksum.accumulate(char)
            status.parseState = .gotSystemId
            
        case .gotSystemId:
            rxpack.componentId = char
            rxpack.checksum.accumulate(char)
            status.parseState = .gotComponentId
            
        case .gotComponentId:
            // Check Message length if `checkMessageLength` enabled and
            // `messageLengths` contains proper id. If `messageLengths` does not
            // contain info for current messageId, parsing will fail later on CRC check.
            if checkMessageLength {
                let messageLength = messageLengths[char] ?? 0
                if rxpack.length != messageLength {
                    status.parseError += 1
                    status.parseState = .idle
                    let error = ParseError.invalidPayloadLength(messageId: char, receivedLength: rxpack.length, expectedLength: messageLength)
                    delegate?.didFailToReceive(packet: nil, with: error, on: channel, via: self)
                    break
                }
            }
            
            rxpack.messageId = char
            rxpack.checksum.accumulate(char)
            
            if rxpack.length == 0 {
                status.parseState = .gotPayload
            } else {
                status.parseState = .gotMessageId
            }
            
        case .gotMessageId:
            rxpack.payload.append(char)
            rxpack.checksum.accumulate(char)
            
            if rxpack.payload.count == Int(rxpack.length) {
                status.parseState = .gotPayload
            }
            
        case .gotPayload:
            if crcExtra && (messageCRCsExtra[rxpack.messageId] != nil) {
                rxpack.checksum.accumulate(messageCRCsExtra[rxpack.messageId]!)
            }
            
            rxpack.payload.append(char)
            
            if char != rxpack.checksum.lowByte {
                status.parseState = .gotBadCRC1
                fallthrough
            } else {
                status.parseState = .gotCRC1
            }
            
        case .gotCRC1, .gotBadCRC1:
            if (status.parseState == .gotBadCRC1) || (char != rxpack.checksum.highByte) {
                status.parseError += 1
                status.packetReceived = .badCRC
                
                let error = messageIdToClass[rxpack.messageId] == nil ? ParseError.unknownMessageId(messageId: rxpack.messageId) : ParseError.badCRC(messageId: rxpack.messageId)
                delegate?.didFailToReceive(packet: Packet(packet: rxpack), with: error, on: channel, via: self)
                handleSTX(char: char, rxpack: rxpack, status: status)
            } else {
                // Successfully got message
                rxpack.payload.append(char)
                status.packetReceived = .ok
            }
            status.parseState = .idle
        }
        
        defer {
            // Сollect stat here
            
            status.parseError = 0
        }
        
        // If a packet has been successfully received
        guard status.packetReceived == .ok else {
            return nil
        }
        
        // Copy and delegate received packet
        let packet = Packet(packet: rxpack)
        delegate?.didReceive(packet: packet, on: channel, via: self)
        
        status.currentRxSeq = rxpack.sequence
        // Initial condition: If no packet has been received so far, drop count is undefined
        if status.packetRxSuccessCount == 0 {
            status.packetRxDropCount = 0
        }
        // Count this packet as received
        status.packetRxSuccessCount = status.packetRxSuccessCount &+ 1
        
        // Try to create appropriate Message structure, delegate results
        guard let messageClass = messageIdToClass[packet.messageId] else {
            let error = ParseError.unknownMessageId(messageId: rxpack.messageId)
            delegate?.didFailToParseMessage(from: packet, with: error, on: channel, via: self)
            return packet
        }
        
        do {
            packet.message = try messageClass.init(data: rxpack.payload)
            delegate?.didParse(message: packet.message!, from: packet, on: channel, via: self)
        } catch {
            delegate?.didFailToParseMessage(from: packet, with: error as! MAVLinkError, on: channel, via: self)
            return packet
        }
        
        return packet
    }
    
    /// Parse new portion of data, then call `messageHandler` if new message
    /// is available.
    ///
    /// - parameter data:           Data to be parsed.
    /// - parameter channel:        Id of the current channel. This allows to
    /// parse different channels with this function. A channel is not a physical
    /// message channel like a serial port, but a logic partition of the
    /// communication streams in this case.
    /// - parameter messageHandler: The message handler to call when the
    /// provided data is enough to complete message parsing. Unless you have
    /// provided a custom delegate, this parameter must not be `nil`, because
    /// there is no other way to retrieve the parsed message and packet.
    public func parse(data: Data, channel: Channel, messageHandler: ((Message, Packet) -> Void)? = nil) {
        data.forEach { byte in
            if let packet = parse(char: byte, channel: channel), let message = packet.message, let messageHandler = messageHandler {
                messageHandler(message, packet)
            }
        }
    }
    
    /// Prepare `message` bytes for sending, pass to `delegate` for further
    /// processing and increase sequence counter.
    ///
    /// - parameter message:     Message to be compiled into bytes and sent.
    /// - parameter systemId:    Id of the sending (this) system.
    /// - parameter componentId: Id of the sending component.
    /// - parameter channel:     Id of the current channel.
    ///
    /// - throws: Throws `PackError`.
    public func dispatch(message: Message, systemId: UInt8, componentId: UInt8, channel: Channel) throws {
        let channelStatus = channelStatuses[Int(channel)]
        let packet = Packet(message: message, systemId: systemId, componentId: componentId, channel: channel)
        let data = try packet.finalize(sequence: channelStatus.currentTxSeq)
        delegate?.didFinalize(message: message, from: packet, to: data, on: channel, in: self)
        channelStatus.currentTxSeq = channelStatus.currentTxSeq &+ 1
    }
}

/// MAVLink Packet structure to store received data that is not full message yet.
/// Contains additional to Message info like channel, system id, component id
/// and raw payload data, etc. Also used to store and transfer received data of
/// unknown or corrupted Messages.
/// [More details](https://mavlink.io/en).
public class Packet {
    
    /// MAVlink Packet constants
    struct Constant {
        
        /// Maximum packets payload length
        static let maxPayloadLength = UInt8.max
        
        static let numberOfChecksumBytes = 2
        
        /// Length of core header (of the comm. layer): message length
        /// (1 byte) + message sequence (1 byte) + message system id (1 byte) +
        /// message component id (1 byte) + message type id (1 byte).
        static let coreHeaderLength = 5
        
        /// Length of all header bytes, including core and checksum
        static let numberOfHeaderBytes = Constant.numberOfChecksumBytes + Constant.coreHeaderLength + 1
        
        /// Packet start sign. Indicates the start of a new packet. v1.0.
        static let packetStx: UInt8 = 0xFE
    }
    
    /// Channel on which packet was received
    public internal(set) var channel: UInt8 = 0
    
    /// Sent at the end of packet
    public internal(set) var checksum = Checksum()
    
    /// Protocol magic marker (PacketStx value)
    public internal(set) var magic: UInt8 = 0
    
    /// Length of payload
    public internal(set) var length: UInt8 = 0
    
    /// Sequence of packet
    public internal(set) var sequence: UInt8 = 0
    
    /// Id of message sender system/aircraft
    public internal(set) var systemId: UInt8 = 0
    
    /// Id of the message sender component
    public internal(set) var componentId: UInt8 = 0
    
    /// Id of message type in payload
    public internal(set) var messageId: UInt8 = 0
    
    /// Message bytes
    public internal(set) var payload = Data(capacity: Int(Constant.maxPayloadLength) + Constant.numberOfChecksumBytes)
    
    /// Received Message structure if available
    public internal(set) var message: Message?
    
    /// Initialize copy of provided Packet.
    ///
    /// - parameter packet: Packet to copy
    init(packet: Packet) {
        channel = packet.channel
        checksum = packet.checksum
        magic = packet.magic
        length = packet.length
        sequence = packet.sequence
        systemId = packet.systemId
        componentId = packet.componentId
        messageId = packet.messageId
        payload = packet.payload
        message = packet.message
    }
    
    /// Initialize packet with provided `message` for sending.
    ///
    /// - parameter message:     Message to send.
    /// - parameter systemId:    Id of the sending (this) system.
    /// - parameter componentId: Id of the sending component.
    /// - parameter channel:     Id of the current channel.
    init(message: Message, systemId: UInt8, componentId: UInt8, channel: Channel) {
        self.magic = Constant.packetStx
        self.systemId = systemId
        self.componentId = componentId
        self.messageId = type(of: message).id
        self.length = type(of: message).payloadLength
        self.message = message
        self.channel = channel
    }
    
    init() { }
    
    /// Finalize a MAVLink packet with sequence assignment. Returns data that
    /// could be sent to the aircraft. This function calculates the checksum and
    /// sets length and aircraft id correctly. It assumes that the packet is
    /// already correctly initialized with appropriate `message`, `length`,
    /// `systemId`, `componentId`.
    /// Could be used to send packets without `MAVLink` object, in this case you
    /// should take care of `sequence` counter manually.
    ///
    /// - parameter sequence: Each channel counts up its send sequence. It allows
    /// to detect packet loss.
    ///
    /// - throws: Throws `PackError`.
    ///
    /// - returns: Data
    public func finalize(sequence: UInt8) throws -> Data {
        guard let message = message else {
            throw PackError.messageNotSet
        }
        
        guard let crcExtra = messageCRCsExtra[messageId] else {
            throw PackError.crcExtraNotFound(messageId: type(of: message).id)
        }
        
        self.sequence = sequence
        
        let coreHeader = [length, sequence, systemId, componentId, messageId]
        let header = [Constant.packetStx] + coreHeader
        let payload = try message.pack()
        
        checksum.start()
        checksum.accumulate(coreHeader)
        checksum.accumulate(payload)
        checksum.accumulate(crcExtra)
        
        let checksumBytes = [checksum.lowByte, checksum.highByte]
        var packetData = Data(capacity: payload.count + Constant.numberOfHeaderBytes)
        packetData.append(header, count: header.count)
        packetData.append(payload)
        packetData.append(checksumBytes, count: checksumBytes.count)
        
        return packetData
    }
}

/// Struct for storing and calculating checksum.
public struct Checksum {
    
    struct Constants {
        static let x25InitCRCValue: UInt16 = 0xFFFF
    }
    
    public var lowByte: UInt8 {
        return UInt8(truncatingIfNeeded: value)
    }
    
    public var highByte: UInt8 {
        return UInt8(truncatingIfNeeded: value >> 8)
    }
    
    public private(set) var value: UInt16 = 0
    
    init() {
        start()
    }
    
    /// Initialize the buffer for the MCRF4XX CRC.
    mutating func start() {
        value = Constants.x25InitCRCValue
    }
    
    /// Accumulate the MCRF4XX CRC by adding one char at a time. The checksum
    /// function adds the hash of one char at a time to the 16 bit checksum
    /// `value` (`UInt16`).
    ///
    /// - parameter char: New char to hash
    mutating func accumulate(_ char: UInt8) {
        var tmp: UInt8 = char ^ UInt8(truncatingIfNeeded: value)
        tmp ^= (tmp << 4)
        value = (UInt16(value) >> 8) ^ (UInt16(tmp) << 8) ^ (UInt16(tmp) << 3) ^ (UInt16(tmp) >> 4)
    }
    
    /// Accumulate the MCRF4XX CRC by adding `buffer` bytes.
    ///
    /// - parameter buffer: Sequence of bytes to hash
    mutating func accumulate<T: Sequence>(_ buffer: T) where T.Iterator.Element == UInt8 {
        buffer.forEach { accumulate($0) }
    }
}

// MARK: - CF independent host system byte order determination

public enum ByteOrder: UInt32 {
    case unknown
    case littleEndian
    case bigEndian
}

public func hostByteOrder() -> ByteOrder {
    var bigAndLittleEndian: UInt32 = (ByteOrder.bigEndian.rawValue << 24) | ByteOrder.littleEndian.rawValue
    
    let firstByte: UInt8 = withUnsafePointer(to: &bigAndLittleEndian) { numberPointer in
        let bufferPointer = numberPointer.withMemoryRebound(to: UInt8.self, capacity: 4) { pointer in
            return UnsafeBufferPointer(start: pointer, count: 4)
        }
        return bufferPointer[0]
    }
    
    return ByteOrder(rawValue: UInt32(firstByte)) ?? .unknown
}

// MARK: - Data extensions

protocol MAVLinkNumber { }

extension UInt8: MAVLinkNumber { }

extension Int8: MAVLinkNumber { }

extension UInt16: MAVLinkNumber { }

extension Int16: MAVLinkNumber { }

extension UInt32: MAVLinkNumber { }

extension Int32: MAVLinkNumber { }

extension UInt64: MAVLinkNumber { }

extension Int64: MAVLinkNumber { }

extension Float: MAVLinkNumber { }

extension Double: MAVLinkNumber { }

/// Methods for getting properly typed field values from received data.
extension Data {
    
    /// Returns number value (integer or floating point) from receiver's data.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter byteOrder: Current system endianness.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: Returns `MAVLinkNumber` (UInt8, Int8, UInt16, Int16, UInt32,
    /// Int32, UInt64, Int64, Float, Double).
    func number<T: MAVLinkNumber>(at offset: Data.Index, byteOrder: ByteOrder = hostByteOrder()) throws -> T {
        let size = MemoryLayout<T>.stride
        let range: Range<Int> = offset ..< offset + size
        
        guard range.upperBound <= count else {
            throw ParseError.valueSizeOutOfBounds(offset: offset, size: size, upperBound: count)
        }
        
        var bytes = subdata(in: range)
        if byteOrder != .littleEndian {
            bytes.reverse()
        }
        
        return bytes.withUnsafeBytes { $0.pointee }
    }
    
    /// Returns typed array from receiver's data.
    ///
    /// - parameter offset:   Offset in receiver's bytes.
    /// - parameter capacity: Expected number of elements in array.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: `Array<T>`
    func array<T: MAVLinkNumber>(at offset: Data.Index, capacity: Int) throws -> [T] {
        var offset = offset
        var array = [T]()
        
        for _ in 0 ..< capacity {
            array.append(try number(at: offset))
            offset += MemoryLayout<T>.stride
        }
        
        return array
    }
    
    /// Returns ASCII String from receiver's data.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter length: Expected length of string to read.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: `String`
    func string(at offset: Data.Index, length: Int) throws -> String {
        let range: Range<Int> = offset ..< offset + length
        
        guard range.upperBound <= count else {
            throw ParseError.valueSizeOutOfBounds(offset: offset, size: length, upperBound: count)
        }
        
        let bytes = subdata(in: range)
        let emptySubSequence = Data.SubSequence(capacity: 0)
        let firstSubSequence = bytes.split(separator: 0x0, maxSplits: 1, omittingEmptySubsequences: false).first ?? emptySubSequence
        
        guard let string = String(bytes: firstSubSequence, encoding: .ascii) else {
            throw ParseError.invalidStringEncoding(offset: offset, length: length)
        }
        
        return string
    }
    
    /// Returns proper typed `Enumeration` subtype value from data or throws
    /// `ParserEnumError` or `ParseError` error.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    ///
    /// - throws: Throws `ParserEnumError`, `ParseError`.
    ///
    /// - returns: Properly typed `Enumeration` subtype value.
    func enumeration<T: Enumeration>(at offset: Data.Index) throws -> T where T.RawValue: MAVLinkNumber {
        let rawValue: T.RawValue = try number(at: offset)
        
        guard let enumerationCase = T(rawValue: rawValue) else {
            throw ParseEnumError.unknownValue(enumType: T.self, rawValue: rawValue, valueOffset: offset)
        }
        
        return enumerationCase
    }

    /// Returns a bitmask that is based on enumeration field. Throws ParseError.
    ///
    /// - parameter offset: Offset in receiver's bytes.
    ///
    /// - throws: Throws `ParseError`.
    ///
    /// - returns: Bitmask subtype value.
    func bitmask<T: MAVLinkBitmask>(at offset: Data.Index) throws -> T where T.RawValue: MAVLinkNumber {
        let rawValue: T.RawValue = try number(at: offset)
        return T(rawValue: rawValue)
    }
}

/// Methods for filling `Data` with properly formatted field values.
extension Data {
    
    /// Sets properly swapped `number` bytes starting from `offset` in
    /// receiver's bytes.
    ///
    /// - warning: Supports only version 1.0 of MAVLink wire protocol
    /// (little-endian byte order).
    ///
    /// - parameter number: Number value to set.
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter byteOrder: Current system endianness.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkNumber>(_ number: T, at offset: Data.Index, byteOrder: ByteOrder = hostByteOrder()) throws {
        let size = MemoryLayout<T>.stride
        let range = offset ..< offset + size
        
        guard range.endIndex <= count else {
            throw PackError.valueSizeOutOfBounds(offset: offset, size: size, upperBound: count)
        }
        
        var number = number
        var bytes: Data = withUnsafePointer(to: &number) { numberPointer in
            let bufferPointer = numberPointer.withMemoryRebound(to: UInt8.self, capacity: size) { pointer in
                return UnsafeBufferPointer(start: pointer, count: size)
            }
            return Data(bufferPointer)
        }
        
        if byteOrder != .littleEndian {
            bytes.reverse()
        }
        
        replaceSubrange(range, with: bytes)
    }
    
    /// Sets `array` of `MAVLinkNumber` values at `offset` with `capacity` validation.
    ///
    /// - parameter array:    Array of values to set.
    /// - parameter offset:   Offset in receiver's bytes.
    /// - parameter capacity: Maximum allowed count of elements in `array`.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkNumber>(_ array: [T], at offset: Data.Index, capacity: Int) throws {
        guard array.count <= capacity else {
            throw PackError.invalidValueLength(offset: offset, providedValueLength: array.count, allowedLength: capacity)
        }
        
        let elementSize = MemoryLayout<T>.stride
        let arraySize = elementSize * array.count
        
        guard offset + arraySize <= count else {
            throw PackError.valueSizeOutOfBounds(offset: offset, size: arraySize, upperBound: count)
        }
        
        for (index, item) in array.enumerated() {
            try set(item, at: offset + index * elementSize)
        }
    }
    
    /// Sets correctly encoded `string` value at `offset` limited to `length` or
    /// throws `PackError`.
    ///
    /// - precondition: `string` value must be ASCII compatible.
    ///
    /// - parameter string: Value to set.
    /// - parameter offset: Offset in receiver's bytes.
    /// - parameter length: Maximum allowed length of `string`.
    ///
    /// - throws: Throws `PackError`.
    mutating func set(_ string: String, at offset: Data.Index, length: Int) throws {
        var bytes = string.data(using: .ascii) ?? Data()
        
        if bytes.isEmpty && string.unicodeScalars.count > 0 {
            throw PackError.invalidStringEncoding(offset: offset, string: string)
        }
        
        // Add optional null-termination if provided string is shorter than
        // expectedlength
        if bytes.count < length {
            bytes.append(0x0)
        }
        
        let asciiCharacters = bytes.withUnsafeBytes { Array(UnsafeBufferPointer<UInt8>(start: $0, count: bytes.count)) }
        try set(asciiCharacters, at: offset, capacity: length)
    }
    
    /// Sets correctly formatted `enumeration` raw value at `offset` or throws
    /// `PackError`.
    ///
    /// - parameter enumeration: Value to set.
    /// - parameter offset:      Offset in receiver's bytes.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: Enumeration>(_ enumeration: T, at offset: Data.Index) throws where T.RawValue: MAVLinkNumber {
        try set(enumeration.rawValue, at: offset)
    }

    /// Sets correctly formatted `bitmask` raw value at `offset` or throws
    /// `PackError`.
    ///
    /// - parameter enumeration: Value to set.
    /// - parameter offset:      Offset in receiver's bytes.
    ///
    /// - throws: Throws `PackError`.
    mutating func set<T: MAVLinkBitmask>(_ enumeration: T, at offset: Data.Index) throws where T.RawValue: MAVLinkNumber {
        try set(enumeration.rawValue, at: offset)
    }
}

// MARK: - Additional MAVLink service info
